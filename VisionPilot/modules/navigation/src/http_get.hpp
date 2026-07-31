#ifndef VISIONPILOT_NAV_HTTP_GET_HPP
#define VISIONPILOT_NAV_HTTP_GET_HPP

#include <string>

namespace navigation {
namespace http {

// Blocking HTTP/1.1 GET over a plain socket. Deliberately tiny: the only server
// it ever talks to is the phone on the same network, so there is no TLS, no
// redirect following and no keep-alive — and therefore no libcurl dependency.
//
// Returns true and fills `body` on a 200. On anything else `body` holds a short
// description of what went wrong.
bool get(const std::string& url, std::string& body, int timeout_ms = 3000);

}  // namespace http
}  // namespace navigation

#endif  // VISIONPILOT_NAV_HTTP_GET_HPP
