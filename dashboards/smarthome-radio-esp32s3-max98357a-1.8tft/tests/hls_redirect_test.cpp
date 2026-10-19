#include "../HlsRedirect.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

static void resolves(const char* base, const char* location, const char* expected) {
  char out[RadioHls::UrlCapacity] = {};
  assert(RadioHls::resolveRedirectUrl(base, location, out, sizeof(out)));
  assert(std::strcmp(out, expected) == 0);
}
static void rejects(const char* base, const char* location) {
  char out[RadioHls::UrlCapacity] = "unchanged";
  assert(!RadioHls::resolveRedirectUrl(base, location, out, sizeof(out)));
  assert(std::strcmp(out, "unchanged") == 0);
}
int main() {
  const char* base = "https://example.org/a/b/live.m3u8?old=1#client";
  assert(RadioHls::isHlsUrl(base));
  assert(RadioHls::isHlsUrl("HTTPS://example.org/LIVE.M3U8?q=.aac#play"));
  assert(!RadioHls::isHlsUrl("https://example.org/live.aac?q=.m3u8"));
  assert(!RadioHls::isHlsUrl("https://example.org/live.m3u8/child"));
  assert(!RadioHls::isHlsUrl("https://example.org/#live.m3u8"));
  assert(!RadioHls::isHlsUrl("https://user@example.org/live.m3u8"));
  assert(!RadioHls::isHlsUrl("live.m3u8"));
  assert(!RadioHls::isHlsUrl(nullptr));

  resolves(base, "https://cdn.example.org/radio/live.m3u8?new=1#player",
           "https://cdn.example.org/radio/live.m3u8?new=1");
  resolves(base, "//cdn.example.org/radio/live.m3u8", "https://cdn.example.org/radio/live.m3u8");
  resolves(base, "/radio/live.m3u8", "https://example.org/radio/live.m3u8");
  resolves(base, "child.m3u8", "https://example.org/a/b/child.m3u8");
  resolves(base, "../child.m3u8?x=1", "https://example.org/a/child.m3u8?x=1");
  resolves(base, "./.././child.m3u8", "https://example.org/a/child.m3u8");
  resolves(base, "../../../../child.m3u8", "https://example.org/child.m3u8");
  resolves(base, "/a//b/../c", "https://example.org/a//c");
  resolves(base, "/a/.", "https://example.org/a/");
  resolves(base, "/a/..", "https://example.org/");
  resolves(base, "?new=1#fragment", "https://example.org/a/b/live.m3u8?new=1");
  resolves(base, "#fragment", "https://example.org/a/b/live.m3u8?old=1");
  resolves("http://example.org", "child.m3u8", "http://example.org/child.m3u8");
  resolves("http://example.org", "https://secure.org", "https://secure.org/");
  resolves(base, "https://[2001:db8::1]:8443/live.m3u8", "https://[2001:db8::1]:8443/live.m3u8");
  resolves(base, "https://example.org:443/live.m3u8", "https://example.org:443/live.m3u8");

  // Captured official Waves redirect. Relative media belongs to the CDN URL,
  // never to the original Waves route (which redirects back to the playlist).
  const char* waves = "https://radio.wavespb.com/live/26ec521ec4f7c684/26ec521ec4f7c684.m3u8";
  const char* cdn = "https://d3hrxqn1tritdh.cloudfront.net/26ec521ec4f7c684/26ec521ec4f7c684.m3u8";
  resolves(waves, cdn, cdn);
  resolves(cdn, "26ec521ec4f7c684-1792385837587.ts",
           "https://d3hrxqn1tritdh.cloudfront.net/26ec521ec4f7c684/26ec521ec4f7c684-1792385837587.ts");
  // Unredirected national Raagam source remains the identical official URL.
  const char* raagam = "https://d3r9rhourdc02u.cloudfront.net/assets/radio/airbang04/air036_Auto.m3u8";
  resolves(raagam, raagam, raagam);

  rejects(base, "http://insecure.org/live.m3u8");
  rejects(base, "hTtP://insecure.org/live.m3u8");
  rejects(base, "https://user:password@example.org/live.m3u8");
  rejects(base, "//user@example.org/live.m3u8");
  rejects(base, "https:///missing-host.m3u8");
  rejects(base, "https://?missing-host");
  rejects(base, "https://example.org:/live.m3u8");
  rejects(base, "https://example.org:65536/live.m3u8");
  rejects(base, "https://example.org:0/live.m3u8");
  rejects(base, "https://example.org:abc/live.m3u8");
  rejects(base, "https://[missing-bracket/live.m3u8");
  rejects(base, "file:///tmp/live.m3u8");
  rejects(base, "javascript:alert(1)");
  rejects(base, "https://example.org/bad path.m3u8");
  rejects(base, "https://example.org/live.m3u8\r\nInjected:1");
  rejects(base, "https://example.org/back\\slash.m3u8");
  rejects(base, ""); rejects(base, nullptr); rejects(nullptr, cdn);
  rejects("ftp://example.org/live.m3u8", cdn);
  const std::string huge = std::string("https://example.org/") + std::string(512, 'a');
  rejects(base, huge.c_str());
  rejects(huge.c_str(), cdn);
  char out[12] = "unchanged";
  assert(!RadioHls::resolveRedirectUrl(base, cdn, out, sizeof(out)));
  assert(std::strcmp(out, "unchanged") == 0);
  assert(!RadioHls::resolveRedirectUrl(base, cdn, nullptr, 512));
  // Successful resolutions can safely replace the current URL in place.
  char alias[RadioHls::UrlCapacity] = "https://example.org/a/live.m3u8";
  assert(RadioHls::resolveRedirectUrl(alias, "../other.m3u8", alias, sizeof(alias)));
  assert(std::strcmp(alias, "https://example.org/other.m3u8") == 0);
  std::puts("HLS redirect policy tests passed");
}
