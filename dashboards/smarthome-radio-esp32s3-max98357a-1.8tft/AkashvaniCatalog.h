#pragma once

#include "RadioModel.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

// The official player publishes HTML location cards and a separate JavaScript
// object of active stream URLs. This parser reads that data without executing
// JavaScript. Its large output buffers belong to the caller (normally PSRAM).
namespace RadioAkashvani {

constexpr std::size_t MaxStations = 512;
constexpr std::size_t MaxLocations = 64;
constexpr std::size_t LocationSize = 64;
constexpr std::size_t MaxHtmlBytes = 1024 * 1024;

struct Entry {
  RadioModel::Station station;
  char location[LocationSize] = {};
};

enum class ParseError : std::uint8_t {
  None, InvalidInput, TooLarge, IncompleteDocument, MalformedHtml,
  MalformedCatalog, MissingCatalog, DuplicateId, UnmatchedId, MissingField,
  FieldTooLong, CapacityExceeded, InvalidStation
};

struct ParseResult {
  ParseError error = ParseError::None;
  std::uint16_t count = 0;
  std::uint16_t locationCount = 0;
  bool ok() const { return error == ParseError::None; }
};

inline const char* errorMessage(ParseError error) {
  switch (error) {
    case ParseError::None: return "Akashvani stations ready";
    case ParseError::InvalidInput: return "Invalid Akashvani input";
    case ParseError::TooLarge: return "Akashvani page too large";
    case ParseError::IncompleteDocument: return "Incomplete Akashvani page";
    case ParseError::MalformedHtml: return "Akashvani HTML changed";
    case ParseError::MalformedCatalog: return "Akashvani catalog changed";
    case ParseError::MissingCatalog: return "Akashvani catalog missing";
    case ParseError::DuplicateId: return "Duplicate Akashvani channel";
    case ParseError::UnmatchedId: return "Akashvani channel mismatch";
    case ParseError::MissingField: return "Akashvani station incomplete";
    case ParseError::FieldTooLong: return "Akashvani field too long";
    case ParseError::CapacityExceeded: return "Akashvani catalog too large";
    case ParseError::InvalidStation: return "Invalid Akashvani stream";
  }
  return "Akashvani parse failed";
}

namespace detail {

struct View {
  const char* data;
  std::size_t size;
  View(const char* text = nullptr, std::size_t length = 0) : data(text), size(length) {}
};

inline bool space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v'; }
inline char lower(char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c + 32) : c; }
inline bool equal(View v, const char* text, bool ignoreCase = false) {
  if (v.size != std::strlen(text)) return false;
  for (std::size_t i = 0; i < v.size; ++i)
    if ((ignoreCase ? lower(v.data[i]) : v.data[i]) != (ignoreCase ? lower(text[i]) : text[i])) return false;
  return true;
}
inline bool prefix(const char* p, const char* end, const char* text, bool ignoreCase = false) {
  const std::size_t n = std::strlen(text);
  return static_cast<std::size_t>(end - p) >= n && equal(View{p, n}, text, ignoreCase);
}
inline const char* find(const char* p, const char* end, const char* text, bool ignoreCase = false) {
  while (p < end) { if (prefix(p, end, text, ignoreCase)) return p; ++p; }
  return nullptr;
}
inline bool append(char* out, std::size_t cap, std::size_t& used, char c) {
  const unsigned char byte = static_cast<unsigned char>(c);
  if (!c || (byte < 0x20 && !space(c)) || byte == 0x7f || used + 1 >= cap) return false;
  out[used++] = c; out[used] = '\0'; return true;
}
inline bool codepoint(char* out, std::size_t cap, std::size_t& used, std::uint32_t c) {
  if (!c || c > 0x10ffff || (c >= 0xd800 && c <= 0xdfff)) return false;
  if (c < 0x80) return append(out, cap, used, static_cast<char>(c));
  if (c < 0x800) return append(out, cap, used, static_cast<char>(0xc0 | (c >> 6))) &&
      append(out, cap, used, static_cast<char>(0x80 | (c & 63)));
  if (c < 0x10000) return append(out, cap, used, static_cast<char>(0xe0 | (c >> 12))) &&
      append(out, cap, used, static_cast<char>(0x80 | ((c >> 6) & 63))) &&
      append(out, cap, used, static_cast<char>(0x80 | (c & 63)));
  return append(out, cap, used, static_cast<char>(0xf0 | (c >> 18))) &&
      append(out, cap, used, static_cast<char>(0x80 | ((c >> 12) & 63))) &&
      append(out, cap, used, static_cast<char>(0x80 | ((c >> 6) & 63))) &&
      append(out, cap, used, static_cast<char>(0x80 | (c & 63)));
}
inline int digit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c = lower(c); return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}
inline bool number(View v, unsigned base, std::uint32_t& value) {
  if (!v.size) return false;
  value = 0;
  for (std::size_t i = 0; i < v.size; ++i) {
    const int n = digit(v.data[i]);
    if (n < 0 || static_cast<unsigned>(n) >= base || value > (0x10ffffu - n) / base) return false;
    value = value * base + n;
  }
  return true;
}
inline bool id(View v, char (&out)[40]) {
  while (v.size && space(*v.data)) { ++v.data; --v.size; }
  while (v.size && space(v.data[v.size - 1])) --v.size;
  if (!v.size || v.size >= sizeof(out)) return false;
  for (std::size_t i = 0; i < v.size; ++i) if (v.data[i] < '0' || v.data[i] > '9') return false;
  while (v.size > 1 && *v.data == '0') { ++v.data; --v.size; }
  if (v.size == 1 && *v.data == '0') return false;
  std::memcpy(out, v.data, v.size); out[v.size] = '\0'; return true;
}
inline void normalize(char* text, bool uppercase) {
  std::size_t n = 0;
  bool gap = false;
  for (std::size_t i = 0; text[i]; ++i) {
    unsigned char c = static_cast<unsigned char>(text[i]);
    if (space(static_cast<char>(c)) || (c == 0xc2 && static_cast<unsigned char>(text[i + 1]) == 0xa0)) {
      if (c == 0xc2) ++i;
      if (n) gap = true;
      continue;
    }
    if (gap) text[n++] = ' ';
    gap = false;
    text[n++] = uppercase && c >= 'a' && c <= 'z' ? static_cast<char>(c - 32) : static_cast<char>(c);
  }
  text[n] = '\0';
}
inline bool htmlText(View v, char* out, std::size_t cap, std::size_t& used) {
  for (std::size_t i = 0; i < v.size; ++i) {
    if (v.data[i] != '&') { if (!append(out, cap, used, v.data[i])) return false; continue; }
    std::size_t j = i + 1;
    while (j < v.size && j - i <= 12 && v.data[j] != ';') ++j;
    // Bare ampersands also occur in the official HTML (Jammu & Kashmir).
    if (j == v.size || v.data[j] != ';') { if (!append(out, cap, used, '&')) return false; continue; }
    View entity{v.data + i + 1, j - i - 1}; std::uint32_t c = 0;
    if (equal(entity, "amp")) c = '&'; else if (equal(entity, "quot")) c = '"';
    else if (equal(entity, "apos")) c = '\''; else if (equal(entity, "lt")) c = '<';
    else if (equal(entity, "gt")) c = '>'; else if (equal(entity, "nbsp")) c = ' ';
    else if (equal(entity, "ndash")) c = 0x2013; else if (equal(entity, "mdash")) c = 0x2014;
    else if (entity.size && entity.data[0] == '#') {
      ++entity.data; --entity.size; unsigned base = 10;
      if (entity.size && lower(entity.data[0]) == 'x') { ++entity.data; --entity.size; base = 16; }
      if (!number(entity, base, c)) return false;
    } else return false;
    if (!codepoint(out, cap, used, c)) return false;
    i = j;
  }
  return true;
}

struct Tag { View name; View attributes; bool closing = false; bool selfClosing = false; };
inline bool tag(const char*& p, const char* end, Tag& result) {
  if (p == end || *p++ != '<') return false;
  if (p < end && *p == '/') { result.closing = true; ++p; }
  const char* begin = p;
  while (p < end && !space(*p) && *p != '>' && *p != '/') ++p;
  result.name = View{begin, static_cast<std::size_t>(p - begin)};
  begin = p; char quote = 0; bool valueStart = false, unquotedValue = false;
  while (p < end) {
    if (quote) { if (*p == quote) quote = 0; }
    else if (*p == '>') {
      const char* last = p;
      while (last > begin && space(last[-1])) --last;
      result.selfClosing = last > begin && last[-1] == '/';
      result.attributes = View{begin, static_cast<std::size_t>(p - begin)}; ++p;
      return result.name.size != 0;
    } else if (valueStart) {
      if (!space(*p)) {
        if (*p == '\'' || *p == '"') quote = *p;
        else unquotedValue = true;
        valueStart = false;
      }
    } else if (space(*p)) {
      unquotedValue = false;
    } else if (*p == '=' && !unquotedValue) {
      valueStart = true;
    }
    ++p;
  }
  return false;
}
inline View attribute(const Tag& t, const char* wanted) {
  const char* p = t.attributes.data; const char* end = p + t.attributes.size;
  View found;
  while (p < end) {
    while (p < end && (space(*p) || *p == '/')) ++p;
    const char* begin = p;
    while (p < end && !space(*p) && *p != '=') ++p;
    View name{begin, static_cast<std::size_t>(p - begin)};
    while (p < end && space(*p)) ++p;
    View value;
    if (p < end && *p == '=') {
      ++p; while (p < end && space(*p)) ++p;
      if (p < end && (*p == '\'' || *p == '"')) {
        const char quote = *p++; begin = p; while (p < end && *p != quote) ++p;
        value = View{begin, static_cast<std::size_t>(p - begin)}; if (p < end) ++p;
      } else {
        begin = p; while (p < end && !space(*p)) ++p;
        value = View{begin, static_cast<std::size_t>(p - begin)};
      }
    }
    if (equal(name, wanted, true)) { if (found.data) return View{}; found = value; }
  }
  return found;
}
inline bool hasClass(const Tag& t, const char* wanted) {
  View classes = attribute(t, "class");
  std::size_t i = 0;
  while (i < classes.size) {
    while (i < classes.size && space(classes.data[i])) ++i;
    const std::size_t start = i;
    while (i < classes.size && !space(classes.data[i])) ++i;
    if (equal(View{classes.data + start, i - start}, wanted)) return true;
  }
  return false;
}

enum class Kind { End, Identifier, String, Number, Punctuation, Error };
struct Token { Kind kind = Kind::End; View value; char punctuation = 0; char quote = 0; };
class Lexer {
 public:
  explicit Lexer(View source) : p_(source.data), end_(source.data + source.size) {}
  Token next() {
    for (;;) {
      while (p_ < end_ && space(*p_)) ++p_;
      if (prefix(p_, end_, "//")) { while (p_ < end_ && *p_ != '\n') ++p_; continue; }
      if (prefix(p_, end_, "/*")) {
        const char* close = find(p_ + 2, end_, "*/");
        if (!close) return failure(); p_ = close + 2; continue;
      }
      break;
    }
    if (p_ == end_) return Token{};
    const char* begin = p_;
    if (*p_ == '\'' || *p_ == '"' || *p_ == '`') {
      const char quote = *p_++; begin = p_;
      while (p_ < end_) {
        if (*p_ == quote) { Token t; t.kind = Kind::String; t.quote = quote; t.value = View{begin, static_cast<std::size_t>(p_++ - begin)}; return t; }
        if (*p_ == '\\') { if (++p_ == end_) return failure(); }
        else if (quote != '`' && (*p_ == '\r' || *p_ == '\n')) return failure();
        ++p_;
      }
      return failure();
    }
    Token t;
    if (identifierStart(*p_)) {
      ++p_; while (p_ < end_ && (identifierStart(*p_) || (*p_ >= '0' && *p_ <= '9'))) ++p_;
      t.kind = Kind::Identifier;
    } else if (*p_ >= '0' && *p_ <= '9') {
      ++p_; while (p_ < end_ && *p_ >= '0' && *p_ <= '9') ++p_; t.kind = Kind::Number;
    } else { t.kind = Kind::Punctuation; t.punctuation = *p_++; }
    t.value = View{begin, static_cast<std::size_t>(p_ - begin)}; return t;
  }
 private:
  const char* p_; const char* end_;
  static bool identifierStart(char c) { return (lower(c) >= 'a' && lower(c) <= 'z') || c == '_' || c == '$'; }
  Token failure() { p_ = end_; Token t; t.kind = Kind::Error; return t; }
};
inline bool punctuation(const Token& t, char c) { return t.kind == Kind::Punctuation && t.punctuation == c; }
inline bool jsText(View v, char* out, std::size_t cap) {
  std::size_t used = 0; out[0] = '\0';
  for (std::size_t i = 0; i < v.size; ++i) {
    if (v.data[i] != '\\') { if (!append(out, cap, used, v.data[i])) return false; continue; }
    if (++i == v.size) return false;
    char c = v.data[i];
    if (c == '\n') continue;
    if (c == '\r') { if (i + 1 < v.size && v.data[i + 1] == '\n') ++i; continue; }
    if (c == 'u' || c == 'x') {
      std::size_t digits = c == 'u' ? 4 : 2;
      if (v.size - i - 1 < digits) return false;
      std::uint32_t value = 0;
      if (!number(View{v.data + i + 1, digits}, 16, value)) return false;
      i += digits;
      if (value >= 0xd800 && value <= 0xdbff) {
        if (v.size - i - 1 < 6 || v.data[i + 1] != '\\' || v.data[i + 2] != 'u') return false;
        std::uint32_t low = 0;
        if (!number(View{v.data + i + 3, 4}, 16, low) || low < 0xdc00 || low > 0xdfff) return false;
        value = 0x10000 + ((value - 0xd800) << 10) + low - 0xdc00; i += 6;
      }
      if (!codepoint(out, cap, used, value)) return false;
    } else {
      if (c == 'n') c = '\n'; else if (c == 'r') c = '\r'; else if (c == 't') c = '\t';
      else if (c == 'b') c = '\b'; else if (c == 'f') c = '\f'; else if (c == 'v') c = '\v';
      else if (c == '0' || (c >= '1' && c <= '9')) return false;
      if (!append(out, cap, used, c)) return false;
    }
  }
  return true;
}
inline bool skipValue(Lexer& lexer, Token t) {
  if (t.kind == Kind::String || t.kind == Kind::Identifier || t.kind == Kind::Number) return true;
  if (!punctuation(t, '{') && !punctuation(t, '[')) return false;
  char closes[16]; std::size_t depth = 1; closes[0] = punctuation(t, '{') ? '}' : ']';
  while (depth) {
    t = lexer.next(); if (t.kind == Kind::End || t.kind == Kind::Error) return false;
    if (punctuation(t, '{') || punctuation(t, '[')) {
      if (depth == sizeof(closes)) return false;
      closes[depth++] = punctuation(t, '{') ? '}' : ']';
    } else if (punctuation(t, '}') || punctuation(t, ']')) {
      if (t.punctuation != closes[depth - 1]) return false; --depth;
    }
  }
  return true;
}
inline int findEntry(Entry* entries, std::size_t count, const char* idText) {
  for (std::size_t i = 0; i < count; ++i) if (!std::strcmp(entries[i].station.id, idText)) return static_cast<int>(i);
  return -1;
}
inline ParseError catalog(Lexer& lexer, Entry* entries, std::size_t count) {
  std::size_t matched = 0;
  Token t = lexer.next();
  while (!punctuation(t, '}')) {
    if (t.kind != Kind::String && t.kind != Kind::Number) return ParseError::MalformedCatalog;
    char text[40] = {}; if (!id(t.value, text)) return ParseError::MalformedCatalog;
    const int index = findEntry(entries, count, text);
    if (index < 0) return ParseError::UnmatchedId;
    Entry& entry = entries[index];
    if (entry.station.name[0]) return ParseError::DuplicateId;
    if (!punctuation(lexer.next(), ':') || !punctuation(lexer.next(), '{')) return ParseError::MalformedCatalog;
    bool name = false, url = false;
    t = lexer.next();
    while (!punctuation(t, '}')) {
      if (t.kind != Kind::String && t.kind != Kind::Identifier) return ParseError::MalformedCatalog;
      const bool isName = equal(t.value, "name"), isUrl = equal(t.value, "live_url");
      if (!punctuation(lexer.next(), ':')) return ParseError::MalformedCatalog;
      t = lexer.next();
      if (isName || isUrl) {
        if (t.kind != Kind::String || t.quote == '`' || (isName ? name : url)) return ParseError::MalformedCatalog;
        char* out = isName ? entry.station.name : entry.station.url;
        const std::size_t cap = isName ? sizeof(entry.station.name) : sizeof(entry.station.url);
        if (!jsText(t.value, out, cap)) return ParseError::FieldTooLong;
        if (isName) { normalize(out, false); name = true; } else url = true;
      } else if (!skipValue(lexer, t)) return ParseError::MalformedCatalog;
      t = lexer.next();
      if (punctuation(t, '}')) break;
      if (!punctuation(t, ',')) return ParseError::MalformedCatalog;
      t = lexer.next();
    }
    if (!name || !url || !entry.station.name[0] || !entry.station.url[0]) return ParseError::MissingField;
    if (!RadioModel::validStation(entry.station)) return ParseError::InvalidStation;
    ++matched;
    t = lexer.next();
    if (punctuation(t, '}')) break;
    if (!punctuation(t, ',')) return ParseError::MalformedCatalog;
    t = lexer.next();
  }
  return matched == count ? ParseError::None : ParseError::UnmatchedId;
}

}  // namespace detail

// Output is a transaction scratch buffer: callers must publish it only when
// ok() is true. Failure never reports a usable partial station/location count.
inline ParseResult parse(const char* html, std::size_t length, Entry* entries,
                         char (*locations)[LocationSize],
                         std::size_t entryCapacity = MaxStations,
                         std::size_t locationCapacity = MaxLocations) {
  using namespace detail;
  ParseResult result;
  const auto fail = [](ParseError error) { ParseResult r; r.error = error; return r; };
  if (!html || !length || !entries || !locations || !entryCapacity || !locationCapacity)
    return fail(ParseError::InvalidInput);
  if (length > MaxHtmlBytes) return fail(ParseError::TooLarge);
  if (entryCapacity > MaxStations) entryCapacity = MaxStations;
  if (locationCapacity > MaxLocations) locationCapacity = MaxLocations;
  if (std::memchr(html, '\0', length)) return fail(ParseError::MalformedHtml);
  const char* p = html; const char* end = html + length;
  View scripts[32]; std::size_t scriptCount = 0, count = 0, locationCount = 0;
  bool bodyClosed = false, htmlClosed = false, row = false, hasLocation = false;
  View locationTag; unsigned locationDepth = 0; std::size_t locationUsed = 0;
  while (p < end) {
    if (*p != '<') {
      const char* begin = p; while (p < end && *p != '<') ++p;
      if (locationDepth && !htmlText(View{begin, static_cast<std::size_t>(p - begin)},
          entries[count].location, LocationSize, locationUsed)) return fail(ParseError::FieldTooLong);
      continue;
    }
    if (prefix(p, end, "<!--")) {
      const char* close = find(p + 4, end, "-->");
      if (!close) return fail(ParseError::IncompleteDocument);
      p = close + 3; continue;
    }
    Tag t;
    if (!tag(p, end, t)) return fail(ParseError::IncompleteDocument);
    if (!t.closing && (equal(t.name, "script", true) || equal(t.name, "style", true))) {
      const bool script = equal(t.name, "script", true);
      const char* close = find(p, end, script ? "</script" : "</style", true);
      if (!close) return fail(ParseError::IncompleteDocument);
      if (script && !attribute(t, "src").data) {
        if (scriptCount == sizeof(scripts) / sizeof(scripts[0])) return fail(ParseError::CapacityExceeded);
        scripts[scriptCount++] = View{p, static_cast<std::size_t>(close - p)};
      }
      p = close; Tag closing;
      if (!tag(p, end, closing) || !closing.closing || !equal(closing.name, script ? "script" : "style", true))
        return fail(ParseError::IncompleteDocument);
      continue;
    }
    if (equal(t.name, "body", true) && t.closing) bodyClosed = true;
    if (equal(t.name, "html", true) && t.closing) htmlClosed = true;
    if (!t.closing && equal(t.name, "li", true) && hasClass(t, "selectchannel")) {
      if (row) return fail(ParseError::MalformedHtml);
      if (count == entryCapacity) return fail(ParseError::CapacityExceeded);
      entries[count] = Entry{};
      if (!id(attribute(t, "data-channel"), entries[count].station.id)) return fail(ParseError::MalformedHtml);
      if (findEntry(entries, count, entries[count].station.id) >= 0) return fail(ParseError::DuplicateId);
      entries[count].station.source = RadioModel::Source::Akashvani;
      RadioModel::copyText(entries[count].station.country, "IN");
      row = true; hasLocation = false; locationUsed = 0;
    }
    if (row && !t.closing && hasClass(t, "channel-state")) {
      if (hasLocation || locationDepth || t.selfClosing) return fail(ParseError::MalformedHtml);
      hasLocation = true; locationTag = t.name; locationDepth = 1;
    } else if (locationDepth && equal(t.name, "br", true) && !t.closing) {
      if (!append(entries[count].location, LocationSize, locationUsed, ' ')) return fail(ParseError::FieldTooLong);
    } else if (locationDepth && t.name.size == locationTag.size) {
      bool same = true;
      for (std::size_t i = 0; i < t.name.size; ++i) if (lower(t.name.data[i]) != lower(locationTag.data[i])) same = false;
      if (same && !t.selfClosing) { if (t.closing) --locationDepth; else ++locationDepth; }
    }
    if (row && equal(t.name, "li", true) && t.closing) {
      if (locationDepth) return fail(ParseError::MalformedHtml);
      normalize(entries[count].location, true);
      if (!hasLocation || !entries[count].location[0]) return fail(ParseError::MissingField);
      bool foundLocation = false;
      for (std::size_t i = 0; i < locationCount; ++i)
        if (!std::strcmp(locations[i], entries[count].location)) foundLocation = true;
      if (!foundLocation) {
        if (locationCount == locationCapacity) return fail(ParseError::CapacityExceeded);
        std::memcpy(locations[locationCount++], entries[count].location, LocationSize);
      }
      ++count; row = false;
    }
  }
  if (!bodyClosed || !htmlClosed || row || locationDepth) return fail(ParseError::IncompleteDocument);
  if (!count) return fail(ParseError::MissingCatalog);
  bool parsed = false;
  for (std::size_t s = 0; s < scriptCount; ++s) {
    Lexer lexer(scripts[s]); Token t;
    while ((t = lexer.next()).kind != Kind::End) {
      if (t.kind == Kind::Error) return fail(ParseError::MalformedCatalog);
      if (t.kind != Kind::Identifier || (!equal(t.value, "var") && !equal(t.value, "let") && !equal(t.value, "const"))) continue;
      t = lexer.next();
      if (t.kind != Kind::Identifier || !equal(t.value, "channels")) continue;
      if (parsed) return fail(ParseError::DuplicateId);
      if (!punctuation(lexer.next(), '=') || !punctuation(lexer.next(), '{')) return fail(ParseError::MalformedCatalog);
      const ParseError error = catalog(lexer, entries, count);
      if (error != ParseError::None) return fail(error);
      parsed = true;
    }
  }
  if (!parsed) return fail(ParseError::MissingCatalog);
  result.count = static_cast<std::uint16_t>(count);
  result.locationCount = static_cast<std::uint16_t>(locationCount);
  return result;
}

}  // namespace RadioAkashvani
