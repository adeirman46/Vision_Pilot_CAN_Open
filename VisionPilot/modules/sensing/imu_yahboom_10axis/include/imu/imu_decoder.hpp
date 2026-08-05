#ifndef IMU_YAHBOOM_IMU_DECODER_HPP
#define IMU_YAHBOOM_IMU_DECODER_HPP

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <imu/imu_types.hpp>

namespace imu {

// Byte-stream decoder. Feed it whatever read() returned, pop whole samples.
//
// The Yahboom 10-axis modules are sold with more than one firmware, so rather
// than assuming a format the decoder runs every parser it knows over the stream
// and locks onto the first one that produces checksum-clean frames. Once locked
// it stays locked, so a stray 0x55 inside a Yesense payload cannot flip it.
class ImuDecoder
{
public:
    void feed(const uint8_t* data, size_t n);

    // Oldest complete sample, if one is ready.
    bool pop(ImuSample& out);

    Protocol           protocol() const { return protocol_; }
    const StreamStats& stats() const { return stats_; }

    // Pin the parser instead of auto-detecting.
    void force_protocol(Protocol p) { protocol_ = p; }

    void reset();

    // What the sniffer learned about frames it could not decode: repeating
    // header bytes and the byte spacing between them. This is what turns "no
    // data" into a one-line description of the unknown format.
    std::string sniff_report() const;

private:
    bool try_yahboom();
    bool try_witmotion();
    bool try_yesense();
    bool try_ascii();

    // Both the Yahboom and the WitMotion formats split one reading across
    // several frames, so a sample is stitched together and emitted when a
    // channel repeats — that is where the next cycle begins.
    void group_store(uint8_t channel_bit);
    void group_flush();

    void wit_store(uint8_t type, const uint8_t* payload);

    void note_unknown_byte(uint8_t b);

    std::deque<uint8_t>    buf_;
    std::vector<ImuSample> ready_;
    Protocol               protocol_ = Protocol::Unknown;
    StreamStats            stats_;

    ImuSample working_;
    uint32_t  wit_seen_mask_ = 0;   // channel types filled since the last emit
    std::string line_;              // partial ASCII line

    // A single valid-looking frame can appear by chance inside binary noise, so
    // a parser only wins the stream after several clean ones in a row.
    int yah_hits_   = 0;
    int wit_hits_   = 0;
    int yes_hits_   = 0;
    int ascii_hits_ = 0;

    // Sniffer state, only used while the protocol is unknown.
    std::map<uint8_t, uint32_t> byte_hist_;
    uint64_t                    last_emit_us_ = 0;
    double                      hz_avg_       = 0.0;
};

// Human-readable hex+ASCII dump, 16 bytes per line, for --raw.
std::string hex_dump(const uint8_t* data, size_t n, uint64_t offset);

}  // namespace imu

#endif  // IMU_YAHBOOM_IMU_DECODER_HPP
