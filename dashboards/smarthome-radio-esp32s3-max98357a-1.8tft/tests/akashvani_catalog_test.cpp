#include "../AkashvaniCatalog.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

using namespace RadioAkashvani;

struct Buffers {
  std::unique_ptr<Entry[]> entries{new Entry[MaxStations]};
  std::unique_ptr<char[][LocationSize]> locations{new char[MaxLocations][LocationSize]{}};
  ParseResult read(const std::string& html, std::size_t entryCapacity = MaxStations,
                   std::size_t locationCapacity = MaxLocations) {
    return parse(html.data(), html.size(), entries.get(), locations.get(), entryCapacity, locationCapacity);
  }
};

static std::string file(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::binary);
  assert(input.good());
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

static void replace(std::string& text, const std::string& from, const std::string& to) {
  const std::size_t at = text.find(from);
  assert(at != std::string::npos);
  text.replace(at, from.size(), to);
}

static std::string document(const std::string& location, const std::string& name,
                            const std::string& url = "https://audio.example/live.m3u8") {
  return "<html><body><li class='selectchannel' data-channel='4'><p class='channel-state'>" + location +
      "</p></li></body></html><script>const channels={'4':{name:'" + name + "',live_url:'" + url + "'}};</script>";
}

static void fails(Buffers& b, const std::string& html, ParseError error) {
  const ParseResult r = b.read(html);
  if (r.error != error) {
    std::fprintf(stderr, "expected %s, got %s\n", errorMessage(error), errorMessage(r.error));
    std::abort();
  }
  assert(!r.ok() && !r.count && !r.locationCount);
}

static void testFixture(const std::string& fixture) {
  Buffers b;
  const ParseResult r = b.read(fixture);
  assert(r.ok() && r.count == 3 && r.locationCount == 2);
  assert(!std::strcmp(b.entries[0].station.id, "4"));
  assert(!std::strcmp(b.entries[0].station.name, "Raagam"));
  assert(!std::strcmp(b.entries[0].station.url,
      "https://d3r9rhourdc02u.cloudfront.net/assets/radio/airbang04/air036_Auto.m3u8"));
  assert(!std::strcmp(b.entries[1].station.id, "1"));
  assert(!std::strcmp(b.entries[1].station.name, "Vividh Bharati"));
  assert(!std::strcmp(b.entries[2].station.name, "FM Rainbow Goa"));
  assert(!std::strcmp(b.locations[0], "NATIONAL"));
  assert(!std::strcmp(b.locations[1], "JAMMU & KASHMIR"));
  assert(!std::strcmp(b.entries[1].station.url, b.entries[2].station.url));
  for (unsigned i = 0; i < r.count; ++i) {
    assert(b.entries[i].station.source == RadioModel::Source::Akashvani);
    assert(!std::strcmp(b.entries[i].station.country, "IN"));
    assert(!b.entries[i].station.bitrateKbps);
    assert(RadioModel::validStation(b.entries[i].station));
  }
  // Inline data may be moved before the HTML cards without changing the join.
  const std::size_t script = fixture.find("<script");
  const std::string reordered = fixture.substr(script) + fixture.substr(0, script);
  assert(b.read(reordered).ok());
}

static void testEscaping() {
  Buffers b;
  assert(b.read(document("  tamil\n\t nadu &#x26; &#38; &#160; ",
      " O\\'Brien \\u2013 \\uD83C\\uDFB5\\nFM ",
      "https:\\x2f\\u002fexample.com/live.m3u8?x=1&y=2")).ok());
  assert(!std::strcmp(b.entries[0].station.name, "O'Brien \xe2\x80\x93 \xf0\x9f\x8e\xb5 FM"));
  assert(!std::strcmp(b.entries[0].location, "TAMIL NADU & &"));
  assert(!std::strcmp(b.entries[0].station.url, "https://example.com/live.m3u8?x=1&y=2"));
  fails(b, document("NATIONAL", "Invalid\\uD800"), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", "Invalid\\0"), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", "Invalid\\b"), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", "Bad URL", "https://example.com/a\\n.m3u8"), ParseError::InvalidStation);
}

static void testRejections(const std::string& fixture) {
  Buffers b;
  // Exercise every possible interrupted transfer, including quoted attributes,
  // escaped JavaScript strings, comments and the final closing script tag.
  const std::size_t complete = fixture.rfind("</script>") + std::strlen("</script>");
  for (std::size_t cut = 0; cut < complete; ++cut) assert(!b.read(fixture.substr(0, cut)).ok());
  std::string text = fixture;
  replace(text, "</script>", ""); fails(b, text, ParseError::IncompleteDocument);
  text = fixture; replace(text, "</html>", ""); fails(b, text, ParseError::IncompleteDocument);
  text = fixture; replace(text, "</body>", ""); fails(b, text, ParseError::IncompleteDocument);
  text = fixture.substr(0, fixture.find("var channels")); fails(b, text, ParseError::IncompleteDocument);
  text = fixture; replace(text, "</li>", ""); fails(b, text, ParseError::MalformedHtml);
  text = fixture; replace(text, "data-channel='1'", "data-channel='4'"); fails(b, text, ParseError::DuplicateId);
  text = fixture; replace(text, "'1': {", "'4': {"); fails(b, text, ParseError::DuplicateId);
  text = fixture; replace(text, "'1': {", "'999': {"); fails(b, text, ParseError::UnmatchedId);
  text = fixture; replace(text, "live_url: 'https://radio.wavespb.com/live/146ed6ec6dea5a24/146ed6ec6dea5a24.m3u8',",
      "ignored_url: 'https://example.com/a.m3u8',"); fails(b, text, ParseError::MissingField);
  fails(b, document("", "Raagam"), ParseError::MissingField);
  fails(b, document(std::string(LocationSize, 'L'), "Raagam"), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", std::string(128, 'N')), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", "Raagam", "https://example.com/" + std::string(512, 'x')), ParseError::FieldTooLong);
  fails(b, document("NATIONAL", "Raagam", "javascript:alert(1)"), ParseError::InvalidStation);
  text = fixture; replace(text, "\"4\": {", "\"4\": { name: 'duplicate',"); fails(b, text, ParseError::MalformedCatalog);
  text = fixture; replace(text, "\nvar channels =", "\nvar different ="); fails(b, text, ParseError::MissingCatalog);
  text = fixture; replace(text, "name: 'Raagam'", "name: `Raagam ${expression}`"); fails(b, text, ParseError::MalformedCatalog);
  text = fixture; text += "<script>var channels={};</script>"; fails(b, text, ParseError::DuplicateId);
  text = fixture; text += "<!-- unfinished"; fails(b, text, ParseError::IncompleteDocument);
  text = fixture; text[text.find("Raagam")] = '\0'; fails(b, text, ParseError::MalformedHtml);
  fails(b, std::string(MaxHtmlBytes + 1, 'x'), ParseError::TooLarge);
  assert(b.read(fixture, 2).error == ParseError::CapacityExceeded);
  assert(b.read(fixture, MaxStations, 1).error == ParseError::CapacityExceeded);
  assert(parse(nullptr, 1, b.entries.get(), b.locations.get()).error == ParseError::InvalidInput);
  assert(parse(fixture.data(), fixture.size(), nullptr, b.locations.get()).error == ParseError::InvalidInput);
  assert(parse(fixture.data(), fixture.size(), b.entries.get(), nullptr).error == ParseError::InvalidInput);
  assert(b.read(fixture).ok()); // A failed transaction does not poison the next parse.
}

static std::string many(unsigned count, unsigned locations) {
  std::string html = "<html><body>";
  std::string js = "<script>let channels={";
  for (unsigned i = 1; i <= count; ++i) {
    const std::string id = std::to_string(i);
    html += "<li class='selectchannel' data-channel='" + id + "'><p class='channel-state'>L" +
        std::to_string(i % locations) + "</p></li>";
    js += "'" + id + "':{name:'Station " + id + "',live_url:'https://example.com/live.m3u8'},";
  }
  return html + "</body></html>" + js + "};</script>";
}

static void testCapacity() {
  Buffers b;
  const ParseResult r = b.read(many(MaxStations, MaxLocations));
  assert(r.ok() && r.count == MaxStations && r.locationCount == MaxLocations);
  fails(b, many(MaxStations + 1, MaxLocations), ParseError::CapacityExceeded);
  fails(b, many(MaxStations, MaxLocations + 1), ParseError::CapacityExceeded);
}

int main(int argc, char** argv) {
  const std::string source = __FILE__;
  const std::string fixture = file(source.substr(0, source.find_last_of("/\\") + 1) + "fixtures/akashvani-player.html");
  testFixture(fixture); testEscaping(); testRejections(fixture); testCapacity();
  if (argc == 2) {
    Buffers b; const ParseResult r = b.read(file(argv[1]));
    if (!r.ok()) std::fprintf(stderr, "%s\n", errorMessage(r.error));
    assert(r.ok() && r.count == 277 && r.locationCount == 37);
    int raagam = -1;
    for (unsigned i = 0; i < r.count; ++i) if (!std::strcmp(b.entries[i].station.id, "4")) raagam = static_cast<int>(i);
    assert(raagam >= 0 && !std::strcmp(b.entries[raagam].station.name, "Raagam"));
    assert(!std::strcmp(b.entries[raagam].location, "NATIONAL"));
    std::printf("Current official page: %u stations, %u locations, National/Raagam verified\n", r.count, r.locationCount);
  }
  std::puts("Akashvani catalog tests passed");
}
