#include "http_get.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <fcntl.h>

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <poll.h>

namespace navigation {
namespace http {

namespace {

struct Url {
    std::string host;
    std::string port = "80";
    std::string path = "/";
};

bool parse_url(const std::string& url, Url& out)
{
    const std::string prefix = "http://";
    if (url.rfind(prefix, 0) != 0) return false;

    const std::string rest = url.substr(prefix.size());
    const auto slash = rest.find('/');
    const std::string hostport = rest.substr(0, slash);
    if (slash != std::string::npos) out.path = rest.substr(slash);
    if (hostport.empty()) return false;

    const auto colon = hostport.rfind(':');
    if (colon != std::string::npos && hostport.find(']') == std::string::npos) {
        out.host = hostport.substr(0, colon);
        out.port = hostport.substr(colon + 1);
    } else {
        out.host = hostport;
    }
    return !out.host.empty() && !out.port.empty();
}

// connect() with a deadline. A phone that dropped off the network otherwise
// blocks for the kernel's SYN timeout — over a minute — with the poll thread
// stuck inside it.
int connect_timeout(const Url& url, int timeout_ms, std::string& err)
{
    addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* res = nullptr;
    if (getaddrinfo(url.host.c_str(), url.port.c_str(), &hints, &res) != 0 || !res) {
        err = "cannot resolve " + url.host;
        return -1;
    }

    int fd = -1;
    for (addrinfo* p = res; p; p = p->ai_next) {
        fd = ::socket(p->ai_family, p->ai_socktype | SOCK_NONBLOCK, p->ai_protocol);
        if (fd < 0) continue;

        int rc = ::connect(fd, p->ai_addr, p->ai_addrlen);
        if (rc == 0) break;
        if (errno == EINPROGRESS) {
            pollfd pfd{fd, POLLOUT, 0};
            if (::poll(&pfd, 1, timeout_ms) > 0) {
                int soerr = 0;
                socklen_t len = sizeof(soerr);
                if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0)
                    break;
            }
        }
        ::close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        err = "cannot connect to " + url.host + ":" + url.port;
        return -1;
    }

    // Back to blocking, with per-call timeouts. Clearing O_NONBLOCK is the point
    // of this: it was only set so connect() could be bounded by poll(), and a
    // socket left non-blocking makes every recv() return EAGAIN at once.
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int flag = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    return fd;
}

// Undo Transfer-Encoding: chunked. IP Webcam answers with Content-Length, but
// the small HTTP servers phone apps ship vary, so handle both.
std::string dechunk(const std::string& in)
{
    std::string out;
    size_t pos = 0;
    while (pos < in.size()) {
        const auto eol = in.find("\r\n", pos);
        if (eol == std::string::npos) break;
        size_t len = 0;
        try { len = std::stoul(in.substr(pos, eol - pos), nullptr, 16); }
        catch (...) { break; }
        if (len == 0) break;
        pos = eol + 2;
        if (pos + len > in.size()) len = in.size() - pos;
        out.append(in, pos, len);
        pos += len + 2;   // skip the trailing CRLF
    }
    return out;
}

}  // namespace

bool get(const std::string& url, std::string& body, int timeout_ms)
{
    Url u;
    if (!parse_url(url, u)) { body = "not an http:// URL: " + url; return false; }

    std::string err;
    const int fd = connect_timeout(u, timeout_ms, err);
    if (fd < 0) { body = err; return false; }

    const std::string req =
        "GET " + u.path + " HTTP/1.1\r\n"
        "Host: " + u.host + ":" + u.port + "\r\n"
        "User-Agent: VisionPilot/1.0\r\n"
        "Accept: application/json\r\n"
        "Connection: close\r\n\r\n";

    size_t sent = 0;
    while (sent < req.size()) {
        const ssize_t n = ::send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) { ::close(fd); body = "send failed"; return false; }
        sent += static_cast<size_t>(n);
    }

    std::string raw;
    char buf[4096];
    while (true) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n > 0) { raw.append(buf, static_cast<size_t>(n)); continue; }
        if (n == 0) break;                                  // clean close
        if (errno == EINTR) continue;
        break;                                              // timeout or error
    }
    ::close(fd);

    const auto split = raw.find("\r\n\r\n");
    if (split == std::string::npos) { body = "no HTTP response"; return false; }

    const std::string head = raw.substr(0, split);
    body = raw.substr(split + 4);

    // "HTTP/1.1 200 OK"
    const auto sp = head.find(' ');
    const int code = (sp == std::string::npos)
                         ? 0
                         : std::atoi(head.c_str() + sp + 1);
    if (code != 200) { body = "HTTP " + std::to_string(code); return false; }

    std::string lower = head;
    for (auto& c : lower) c = static_cast<char>(::tolower(c));
    if (lower.find("transfer-encoding: chunked") != std::string::npos)
        body = dechunk(body);

    return true;
}

}  // namespace http
}  // namespace navigation
