#pragma once

namespace ANSIColors {
constexpr const char *reset  = "\x1b[0m";
constexpr const char *red    = "\x1b[31m";
constexpr const char *green  = "\x1b[32m";
constexpr const char *yellow = "\x1b[33m";

constexpr const char *redb    = "\x1b[31;1m"; ///< Red bold
constexpr const char *greenb  = "\x1b[32;1m"; ///< Green bold
constexpr const char *yellowb = "\x1b[33;1m"; ///< Yellow bold
}  // namespace ANSIColors