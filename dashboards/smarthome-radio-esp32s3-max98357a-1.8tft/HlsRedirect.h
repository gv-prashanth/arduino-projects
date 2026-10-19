#pragma once

#include <cstddef>
#include <cstring>

namespace RadioHls {
// Stream URLs must fit the firmware's existing 512-byte Station.url field.
constexpr std::size_t UrlCapacity = 512;
namespace detail {
inline char lower(char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c + 32) : c; }
inline bool prefix(const char* text, std::size_t length, const char* value) {
  const std::size_t n = std::strlen(value);
  if (length < n) return false;
  for (std::size_t i = 0; i < n; ++i) if (lower(text[i]) != value[i]) return false;
  return true;
}
inline bool textLength(const char* text, std::size_t& length) {
  if (!text) return false;
  for (length = 0; length < UrlCapacity; ++length) {
    const unsigned char c = static_cast<unsigned char>(text[length]);
    if (!c) return true;
    if (c <= 0x20 || c == 0x7f || c == '\\') return false;
  }
  return false;
}
struct Url {
  bool secure = false;
  const char* authority = nullptr;
  std::size_t authoritySize = 0;
  const char* path = nullptr;
  std::size_t pathSize = 0;
  const char* query = nullptr; // Includes '?' when present; fragment excluded.
  std::size_t querySize = 0;
};
inline bool host(const char* authority, std::size_t size) {
  if (!size) return false;
  std::size_t port = size;
  if (authority[0] == '[') {
    std::size_t close = 1;
    bool colon = false;
    for (; close < size && authority[close] != ']'; ++close) {
      const char c = lower(authority[close]);
      if (c == ':') colon = true;
      else if (!(c >= '0' && c <= '9') && !(c >= 'a' && c <= 'f') && c != '.') return false;
    }
    if (close == size || close == 1 || !colon) return false;
    if (close + 1 < size) {
      if (authority[close + 1] != ':') return false;
      port = close + 2;
    }
  } else {
    std::size_t name = 0;
    for (; name < size && authority[name] != ':'; ++name) {
      const char c = lower(authority[name]);
      if (!(c >= 'a' && c <= 'z') && !(c >= '0' && c <= '9') && c != '-' && c != '.') return false;
    }
    if (!name || authority[0] == '.' || authority[0] == '-' || authority[name - 1] == '-') return false;
    if (name < size) port = name + 1;
  }
  if (port < size) {
    unsigned value = 0;
    for (std::size_t i = port; i < size; ++i) {
      if (authority[i] < '0' || authority[i] > '9') return false;
      value = value * 10 + static_cast<unsigned>(authority[i] - '0');
      if (value > 65535) return false;
    }
    if (!value) return false;
  } else if (authority[size - 1] == ':') return false;
  return true;
}
inline bool parse(const char* text, std::size_t size, Url& url) {
  std::size_t begin;
  if (prefix(text, size, "https://")) { url.secure = true; begin = 8; }
  else if (prefix(text, size, "http://")) { url.secure = false; begin = 7; }
  else return false;
  std::size_t end = begin;
  while (end < size && text[end] != '/' && text[end] != '?' && text[end] != '#') ++end;
  if (!host(text + begin, end - begin)) return false;
  url.authority = text + begin; url.authoritySize = end - begin;
  begin = end;
  while (end < size && text[end] != '?' && text[end] != '#') ++end;
  url.path = text + begin; url.pathSize = end - begin;
  if (end < size && text[end] == '?') {
    begin = end;
    while (end < size && text[end] != '#') ++end;
    url.query = text + begin; url.querySize = end - begin;
  }
  return true;
}
inline bool append(char* out, std::size_t& used, const char* text, std::size_t size) {
  if (size >= UrlCapacity || used >= UrlCapacity - size) return false;
  if (size) std::memcpy(out + used, text, size);
  used += size; out[used] = '\0'; return true;
}
inline void removeLastSegment(char* out, std::size_t& used) {
  while (used && out[used - 1] != '/') --used;
  if (used) --used;
  out[used] = '\0';
}
// RFC 3986 dot-segment removal on an absolute path; repeated slashes are kept.
inline bool path(const char* input, std::size_t size, char (&out)[UrlCapacity]) {
  std::size_t at = 0, used = 0;
  out[0] = '\0';
  while (at < size) {
    const std::size_t left = size - at;
    if (left >= 3 && std::memcmp(input + at, "/./", 3) == 0) { at += 2; continue; }
    if (left == 2 && std::memcmp(input + at, "/.", 2) == 0) {
      at += 2; if (!append(out, used, "/", 1)) return false; continue;
    }
    if (left >= 4 && std::memcmp(input + at, "/../", 4) == 0) {
      at += 3; removeLastSegment(out, used); continue;
    }
    if (left == 3 && std::memcmp(input + at, "/..", 3) == 0) {
      at += 3; removeLastSegment(out, used);
      if (!append(out, used, "/", 1)) return false; continue;
    }
    std::size_t end = at + 1;
    while (end < size && input[end] != '/') ++end;
    if (!append(out, used, input + at, end - at)) return false;
    at = end;
  }
  return used || append(out, used, "/", 1);
}
} // namespace detail

inline bool isHlsUrl(const char* text) {
  std::size_t size = 0;
  detail::Url url;
  if (!detail::textLength(text, size) || !detail::parse(text, size, url) || url.pathSize < 5) return false;
  return detail::prefix(url.path + url.pathSize - 5, 5, ".m3u8");
}

// Resolve one HTTP Location against its current response URL. No heap, network
// or station mutation. Output is unchanged on failure and may alias either input.
// Fragments are removed because HTTP requests do not send them. HTTPS never
// downgrades, and credentials/controls/backslashes/oversize URLs are rejected.
inline bool resolveRedirectUrl(const char* current, const char* location,
                               char* out, std::size_t capacity) {
  if (!out || !capacity) return false;
  std::size_t currentSize = 0, locationSize = 0;
  detail::Url base;
  if (!detail::textLength(current, currentSize) || !detail::parse(current, currentSize, base) ||
      !detail::textLength(location, locationSize) || !locationSize) return false;
  char combined[UrlCapacity] = {};
  std::size_t used = 0;
  const bool absolute = detail::prefix(location, locationSize, "http://") || detail::prefix(location, locationSize, "https://");
  if (absolute) {
    if (!detail::append(combined, used, location, locationSize)) return false;
  } else if (locationSize >= 2 && location[0] == '/' && location[1] == '/') {
    if (!detail::append(combined, used, base.secure ? "https:" : "http:", base.secure ? 6 : 5) ||
        !detail::append(combined, used, location, locationSize)) return false;
  } else {
    // A colon before any path/query/fragment delimiter denotes another scheme.
    for (std::size_t i = 0; i < locationSize && location[i] != '/' && location[i] != '?' && location[i] != '#'; ++i)
      if (location[i] == ':') return false;
    if (!detail::append(combined, used, base.secure ? "https://" : "http://", base.secure ? 8 : 7) ||
        !detail::append(combined, used, base.authority, base.authoritySize)) return false;
    if (location[0] == '?' || location[0] == '#') {
      if (!detail::append(combined, used, base.pathSize ? base.path : "/", base.pathSize ? base.pathSize : 1)) return false;
      if (location[0] == '#' && !detail::append(combined, used, base.query, base.querySize)) return false;
    } else if (location[0] != '/') {
      std::size_t directorySize = base.pathSize;
      while (directorySize && base.path[directorySize - 1] != '/') --directorySize;
      if (!detail::append(combined, used, directorySize ? base.path : "/", directorySize ? directorySize : 1)) return false;
    }
    if (!detail::append(combined, used, location, locationSize)) return false;
  }
  detail::Url target;
  if (!detail::parse(combined, used, target) || (base.secure && !target.secure)) return false;
  char normalized[UrlCapacity] = {};
  if (!detail::path(target.path, target.pathSize, normalized)) return false;
  char resolved[UrlCapacity] = {};
  used = 0;
  if (!detail::append(resolved, used, target.secure ? "https://" : "http://", target.secure ? 8 : 7) ||
      !detail::append(resolved, used, target.authority, target.authoritySize) ||
      !detail::append(resolved, used, normalized, std::strlen(normalized)) ||
      !detail::append(resolved, used, target.query, target.querySize) || used >= capacity) return false;
  std::memcpy(out, resolved, used + 1);
  return true;
}
} // namespace RadioHls
