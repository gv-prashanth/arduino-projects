#include "HlsStreamResolver.h"

#include <time.h>
#include "HlsRedirect.h"
#include "CatalogTransport.h"

namespace RadioStreaming {
bool resolvePlaybackUrl(const RadioModel::Station& station, char (&out)[512], char (&error)[80]) {
  error[0] = '\0';
  out[0] = '\0';
  if (!RadioModel::copyText(out, station.url)) {
    strlcpy(error, "Stream URL is too long", sizeof(error));
    return false;
  }
  if (!RadioHls::isHlsUrl(station.url)) return true;

  // The audio library retains the initial URL as the HLS relative-path base
  // across HTTP redirects. Resolve the playlist first, then give it the final
  // URL. Never rewrite the catalog or persist this temporary CDN address.
  char current[512] = {};
  if (!RadioHls::resolveRedirectUrl(station.url, station.url, current, sizeof(current))) {
    strlcpy(error, "Invalid HLS playlist URL", sizeof(error));
    return false;
  }
  const uint32_t startedAt = millis();
  for (unsigned redirects = 0; redirects <= 3; ++redirects) {
    if (millis() - startedAt > 20000) {
      strlcpy(error, "HLS redirect resolution timed out", sizeof(error));
      return false;
    }
    const bool secure = RadioModel::detail::startsWithIgnoreCase(current, "https://");
    if (secure && time(nullptr) < 1700000000) {
      const uint32_t clockAt = millis();
      while (time(nullptr) < 1700000000 && millis() - clockAt < 8000)
        vTaskDelay(pdMS_TO_TICKS(100));
      if (time(nullptr) < 1700000000) {
        strlcpy(error, "Waiting for network clock (HLS TLS)", sizeof(error));
        return false;
      }
    }
    WiFiClient plain;
    WiFiClientSecure tls;
    HTTPClient http;
    RadioCatalog::configureHttp(tls, http);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if (!http.begin(secure ? static_cast<NetworkClient&>(tls) : static_cast<NetworkClient&>(plain), current)) {
      strlcpy(error, "Cannot start HLS playlist request", sizeof(error));
      return false;
    }
    http.addHeader("Accept-Encoding", "identity");
    // Only headers are needed. end() closes the response without buffering its
    // body, even if the target is a continuous stream rather than a playlist.
    const int code = http.GET();
    if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
      http.end();
      RadioModel::copyText(out, current);
      if (redirects) Serial.printf("audio: HLS resolved %u redirect(s) for %s\n", redirects, station.name);
      return true;
    }
    if (code == 301 || code == 302 || code == 303 || code == 307 || code == 308) {
      char next[512] = {};
      const bool valid = RadioHls::resolveRedirectUrl(current, http.getLocation().c_str(), next, sizeof(next));
      http.end();
      if (!valid) {
        strlcpy(error, "Invalid or oversized HLS redirect", sizeof(error));
        return false;
      }
      if (redirects == 3) {
        strlcpy(error, "Too many HLS playlist redirects", sizeof(error));
        return false;
      }
      Serial.printf("audio: HLS playlist redirect HTTP %d for %s\n", code, station.name);
      RadioModel::copyText(current, next);
      continue;
    }
    http.end();
    snprintf(error, sizeof(error), "HLS playlist HTTP/network error %d", code);
    return false;
  }
  return false;
}
} // namespace RadioStreaming
