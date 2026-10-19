#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "NetworkJsonReader.h"

// The pinned ESP32 core ships the full Mozilla certificate bundle.
extern const uint8_t radioBundleStart[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t radioBundleEnd[] asm("_binary_x509_crt_bundle_end");

namespace RadioCatalog {
struct NetworkClock {
  uint32_t now() const { return millis(); }
  void pause() const { vTaskDelay(1); }
};

inline void configureHttp(WiFiClientSecure& client, HTTPClient& http) {
  client.setCACertBundle(radioBundleStart, radioBundleEnd - radioBundleStart);
  client.setHandshakeTimeout(5);
  http.setConnectTimeout(3000);
  http.setTimeout(5000);
  http.setReuse(false);
  http.useHTTP10(true); // The body reader requires decoded, unchunked bytes.
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setRedirectLimit(3);
  http.setUserAgent("GTS-Radio/3.0 (ESP32-S3)");
}

inline String urlEncode(const char* value) {
  String encoded;
  const char* hex = "0123456789ABCDEF";
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value); *p; ++p) {
    if (isalnum(*p) || *p == '-' || *p == '_' || *p == '.') encoded += char(*p);
    else { encoded += '%'; encoded += hex[*p >> 4]; encoded += hex[*p & 15]; }
  }
  return encoded;
}
} // namespace RadioCatalog
