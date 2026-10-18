#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace RadioWifi {
namespace Detail {
constexpr std::size_t MaxFormBody = 512;
struct FormFields { char ssid[33] = {}, password[65] = {}, csrf[33] = {}; };
inline int hexDigit(unsigned char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
inline bool decodeComponent(const char* input, std::size_t length, char* output, std::size_t capacity) {
  std::size_t written = 0;
  if (!capacity) return false;
  output[0] = 0;
  for (std::size_t i = 0; i < length; ++i) {
    unsigned char c = input[i];
    if (c == '%') {
      if (i + 2 >= length) return false;
      const int high = hexDigit(input[i + 1]), low = hexDigit(input[i + 2]);
      if (high < 0 || low < 0) return false;
      c = static_cast<unsigned char>((high << 4) | low);
      i += 2;
    } else if (c == '+') c = ' ';
    if (!c || written + 1 >= capacity) return false;
    output[written++] = static_cast<char>(c);
  }
  output[written] = 0;
  return true;
}
inline bool parseForm(const char* body, std::size_t length, FormFields& out) {
  out = {};
  if (!length || length > MaxFormBody || std::memchr(body, 0, length)) return false;
  unsigned fields = 0;
  for (std::size_t from = 0; from < length;) {
    std::size_t end = from, equal = from;
    while (end < length && body[end] != '&') ++end;
    while (equal < end && body[equal] != '=') ++equal;
    if (equal == end) return false;
    unsigned field;
    char* target;
    std::size_t capacity;
    const std::size_t keyLength = equal - from;
    if (keyLength == 4 && !std::memcmp(body + from, "ssid", 4)) {
      field = 1; target = out.ssid; capacity = sizeof(out.ssid);
    } else if (keyLength == 8 && !std::memcmp(body + from, "password", 8)) {
      field = 2; target = out.password; capacity = sizeof(out.password);
    } else if (keyLength == 4 && !std::memcmp(body + from, "csrf", 4)) {
      field = 4; target = out.csrf; capacity = sizeof(out.csrf);
    } else return false;
    if ((fields & field) || !decodeComponent(body + equal + 1, end - equal - 1, target, capacity)) return false;
    fields |= field;
    if (end < length && end + 1 == length) return false;
    from = end + 1;
  }
  return fields == 7;
}
inline bool tokenMatches(const char* expected, const char* candidate, std::size_t length) {
  if (length != 32 || std::strlen(expected) != 32) return false;
  unsigned difference = 0;
  for (std::size_t i = 0; i < 32; ++i) difference |= static_cast<unsigned char>(expected[i] ^ candidate[i]);
  return difference == 0;
}
inline bool cookieMatches(const char* cookie, std::size_t length, const char* token) {
  if (length > 256) return false;
  for (std::size_t from = 0; from < length;) {
    std::size_t end = from;
    while (end < length && cookie[end] != ';') ++end;
    std::size_t first = from, last = end;
    while (first < last && (cookie[first] == ' ' || cookie[first] == '\t')) ++first;
    while (last > first && (cookie[last - 1] == ' ' || cookie[last - 1] == '\t')) --last;
    if (last - first == 44 && !std::memcmp(cookie + first, "gts_session=", 12) &&
        tokenMatches(token, cookie + first + 12, 32)) return true;
    from = end + 1;
  }
  return false;
}
inline bool escapeHtml(const char* input, char* output, std::size_t capacity) {
  if (!capacity) return false;
  std::size_t used = 0;
  for (std::size_t i = 0; input[i]; ++i) {
    const char* replacement = nullptr;
    switch (input[i]) {
      case '&': replacement = "&amp;"; break;
      case '<': replacement = "&lt;"; break;
      case '>': replacement = "&gt;"; break;
      case '"': replacement = "&quot;"; break;
      case '\'': replacement = "&#39;"; break;
    }
    const std::size_t length = replacement ? std::strlen(replacement) : 1;
    if (used + length >= capacity) { output[0] = 0; return false; }
    if (replacement) std::memcpy(output + used, replacement, length);
    else output[used] = input[i];
    used += length;
  }
  output[used] = 0;
  return true;
}
}  // namespace Detail
}  // namespace RadioWifi

#ifndef RADIO_WIFI_HOST_TEST
#include "SettingsService.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_random.h>
#include <ctype.h>
#include <new>
#include <time.h>

namespace RadioWifi {

// Main-loop owner only. No scans, connection waits, or NVS writes occur here.
class WifiManager {
 public:
  bool begin(const RadioStorage::WifiCredentials& credentials,
             const char* fallbackSsid, const char* fallbackPassword) {
    if (begun_) return false;
    begun_ = true;
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);  // Retries below have an explicit deadline.
    const bool initialized = WiFi.mode(WIFI_STA);
    const char* headers[] = {"Cookie", "Origin", "Content-Type"};
    server_.collectHeaders(headers, 3);
    server_.enableDelay(false);
    SaveHandler* save = new (std::nothrow) SaveHandler(*this);
    if (!save) { setStatus("Wi-Fi setup memory unavailable"); return false; }
    server_.addHandler(save);  // WebServer owns this handler.
    server_.on("/", HTTP_GET, [this] { showForm(); });
    server_.onNotFound([this] { reply(404, "Use the setup address shown on the radio."); });

    if (acceptable(credentials)) {
      current_ = credentials;
      haveCredentials_ = true;
    } else if (copyCredentials(current_, fallbackSsid, fallbackPassword) &&
               !placeholder(current_.ssid) && !placeholder(current_.password)) {
      haveCredentials_ = true;
    }
    if (!haveCredentials_) current_ = {};
    const uint32_t now = millis();
    retryFrom_ = now;
    if (haveCredentials_) startAttempt(now);
    else startPortal();
    if (!initialized) setStatus("Wi-Fi initialization failed; retrying");
    return initialized;
  }

  void loop() {
    if (!begun_) return;
    if (portal_) server_.handleClient();
    const uint32_t now = millis();
    if (!portal_ && portalRequested_ && now - portalRetryFrom_ >= kMaxBackoffMs) startPortal();
    if (staged_) {
      current_ = stagedCredentials_;
      stagedCredentials_ = {};
      staged_ = false;
      haveCredentials_ = true;
      backoffMs_ = kFirstBackoffMs;
      online_ = false;
      startAttempt(now);
      credentialsReady_ = true;  // The main loop can now queue the NVS write.
    }

    // A stale connection to a previous SSID must not validate a new form save.
    const bool connected = WiFi.status() == WL_CONNECTED && haveCredentials_ &&
                           WiFi.SSID() == current_.ssid;
    if (connected && (!attempting_ || now - attemptFrom_ >= 250)) {
      if (!online_) {
        online_ = true;
        attempting_ = false;
        backoffMs_ = kFirstBackoffMs;
        connectedAt_ = now;
        closeAfterOnline_ = portal_;
        configTime(0, 0, "pool.ntp.org", "time.google.com");  // UTC; never wait for SNTP.
        setStatus(portal_ ? "Connected; setup closes in 20s" : "Wi-Fi connected");
      }
      if (portal_ && closeAfterOnline_ && now - connectedAt_ >= kPortalGraceMs) closePortal();
      return;
    }
    if (online_) {
      online_ = false;
      attempting_ = false;
      retryFrom_ = now;
      retryDelayMs_ = 0;
      setStatus(portal_ ? "Wi-Fi lost; setup remains open" : "Wi-Fi lost; reconnecting");
    }
    if (attempting_ && now - attemptFrom_ >= kAttemptMs) {
      WiFi.disconnectAsync(false, false);
      attempting_ = false;
      retryFrom_ = now;
      retryDelayMs_ = backoffMs_;
      backoffMs_ = backoffMs_ < kMaxBackoffMs / 2 ? backoffMs_ * 2 : kMaxBackoffMs;
      if (!portal_) startPortal();
      setStatus(portal_ ? "Wi-Fi failed; setup AP active" : "Wi-Fi failed; retrying setup");
    }
    if (!attempting_ && haveCredentials_ && now - retryFrom_ >= retryDelayMs_) startAttempt(now);
    if (!haveCredentials_ && !portal_ && now - portalRetryFrom_ >= kMaxBackoffMs) startPortal();
  }

  bool online() const { return online_; }
  bool portalActive() const { return portal_; }
  const char* apName() const { return apName_; }
  const char* apPassword() const { return apPassword_; }
  String portalAddress() const { return WiFi.softAPIP().toString(); }
  const char* status() const { return status_; }

  void startPortal() {
    if (!begun_) return;
    if (portal_) return;
    portalRequested_ = true;
    portalRetryFrom_ = millis();
    snprintf(apName_, sizeof(apName_), "GTS-Radio-%04X", static_cast<unsigned>(esp_random() & 0xffff));
    snprintf(apPassword_, sizeof(apPassword_), "%08lx", static_cast<unsigned long>(esp_random()));
    randomToken();
    const bool modeReady = WiFi.mode(WIFI_AP_STA);
    const bool started = modeReady && WiFi.softAP(apName_, apPassword_, 1, 0, 2, false, WIFI_AUTH_WPA2_PSK);
    if (!started) {
      setStatus("Setup AP failed; will retry");
      return;
    }
    portal_ = true;
    // A setup session explicitly opened while online stays available until a
    // submitted connection succeeds. Recovery sessions close after reconnect.
    closeAfterOnline_ = false;
    server_.begin();
    setStatus(online_ ? "Wi-Fi setup AP active" : "Join setup AP; open its address");
  }

  bool pollCredentials(RadioStorage::WifiCredentials& credentials) {
    if (!credentialsReady_) return false;
    credentials = current_;
    credentialsReady_ = false;
    return true;
  }

 private:
  static constexpr uint32_t kAttemptMs = 20000;
  static constexpr uint32_t kFirstBackoffMs = 5000;
  static constexpr uint32_t kMaxBackoffMs = 60000;
  static constexpr uint32_t kPortalGraceMs = 20000;
  static constexpr std::size_t kMaxBody = Detail::MaxFormBody;

  // The pinned WebServer offers a raw-body hook before allocating a normal
  // URL-encoded body. Its generic multipart parser has no public size limit;
  // multipart is never emitted by this form and is rejected by handleSave().
  class SaveHandler : public RequestHandler {
   public:
    explicit SaveHandler(WifiManager& owner) : owner_(owner) {}
    bool canHandle(WebServer&, HTTPMethod method, const String& uri) override {
      if (method != HTTP_POST || uri != "/save") return false;
      owner_.bodyReady_ = false;
      owner_.bodyLength_ = 0;
      memset(owner_.body_, 0, sizeof(owner_.body_));
      return true;
    }
    bool canRaw(WebServer&, const String&) override { return true; }
    void raw(WebServer& server, const String&, HTTPRaw& raw) override {
      if (raw.status == RAW_START) {
        owner_.bodyLength_ = 0;
        owner_.bodyReady_ = false;
        server.client().setTimeout(25);
        const String type = server.header("Content-Type");
        if (server.clientContentLength() <= 0 || server.clientContentLength() > static_cast<int>(kMaxBody) ||
            !type.startsWith("application/x-www-form-urlencoded")) {
          owner_.reply(413, "Use the setup form; request is too large or unsupported.");
          server.client().stop();
        }
        // The raw parser reads a buffer larger than this form. Bound its
        // readBytes tail wait as well as the total accepted body size.
      } else if (raw.status == RAW_WRITE) {
        if (owner_.bodyLength_ + raw.currentSize > kMaxBody) {
          owner_.reply(413, "Setup request is too large.");
          server.client().stop();
          return;
        }
        memcpy(owner_.body_ + owner_.bodyLength_, raw.buf, raw.currentSize);
        owner_.bodyLength_ += raw.currentSize;
      } else if (raw.status == RAW_END) {
        owner_.body_[owner_.bodyLength_] = 0;
        owner_.bodyReady_ = true;
      } else if (raw.status == RAW_ABORTED) {
        owner_.bodyReady_ = false;
        owner_.bodyLength_ = 0;
        memset(owner_.body_, 0, sizeof(owner_.body_));
      }
    }
    bool handle(WebServer&, HTTPMethod, const String&) override {
      owner_.handleSave();
      return true;
    }
   private:
    WifiManager& owner_;
  };

  static bool placeholder(const char* value) {
    return !value || !strcmp(value, "XXXXX") || !strcmp(value, "YYYYY") ||
           !strcmp(value, "YOUR_WIFI_SSID") || !strcmp(value, "YOUR_WIFI_PASSWORD") ||
           !strcmp(value, "CHANGE_ME");
  }
  static bool acceptable(const RadioStorage::WifiCredentials& credentials) {
    if (!RadioStorage::validCredentials(credentials)) return false;
    const std::size_t passwordLength = strlen(credentials.password);
    if (passwordLength && passwordLength < 8) return false;
    if (passwordLength == 64) {
      for (std::size_t i = 0; i < passwordLength; ++i) if (!isxdigit(static_cast<unsigned char>(credentials.password[i]))) return false;
    }
    for (std::size_t i = 0; credentials.ssid[i]; ++i)
      if (static_cast<unsigned char>(credentials.ssid[i]) < 32 || credentials.ssid[i] == 127) return false;
    for (std::size_t i = 0; i < passwordLength; ++i)
      if (static_cast<unsigned char>(credentials.password[i]) < 32 || static_cast<unsigned char>(credentials.password[i]) > 126) return false;
    return true;
  }
  static bool copyCredentials(RadioStorage::WifiCredentials& out, const char* ssid, const char* password) {
    out = {};
    if (!ssid || !password || strnlen(ssid, sizeof(out.ssid)) >= sizeof(out.ssid) ||
        strnlen(password, sizeof(out.password)) >= sizeof(out.password)) return false;
    strlcpy(out.ssid, ssid, sizeof(out.ssid));
    strlcpy(out.password, password, sizeof(out.password));
    return acceptable(out);
  }
  void startAttempt(uint32_t now) {
    WiFi.mode(portal_ ? WIFI_AP_STA : WIFI_STA);
    WiFi.disconnectAsync(false, false);
    WiFi.setMinSecurity(current_.password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN);
    WiFi.begin(current_.ssid, current_.password[0] ? current_.password : nullptr);
    attempting_ = true;
    attemptFrom_ = now;
    setStatus(portal_ ? "Connecting; setup remains open" : "Connecting to Wi-Fi");
  }
  void closePortal() {
    server_.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    portal_ = false;
    portalRequested_ = false;
    closeAfterOnline_ = false;
    memset(token_, 0, sizeof(token_));
    setStatus("Wi-Fi connected");
  }
  void setStatus(const char* value) { strlcpy(status_, value, sizeof(status_)); }
  void randomToken() {
    for (int i = 0; i < 4; ++i)
      snprintf(token_ + i * 8, 9, "%08lx", static_cast<unsigned long>(esp_random()));
  }
  bool localRequest() {
    const String address = portalAddress();
    const String host = server_.hostHeader();
    return portal_ && server_.client().localIP() == WiFi.softAPIP() &&
           (host == address || host == address + ":80");
  }
  void responseHeaders() {
    server_.setContentLength(CONTENT_LENGTH_NOT_SET);
    server_.sendHeader("Cache-Control", "no-store");
    server_.sendHeader("Pragma", "no-cache");
    server_.sendHeader("X-Content-Type-Options", "nosniff");
    server_.sendHeader("Content-Security-Policy", "default-src 'none'; style-src 'unsafe-inline'; form-action 'self'; frame-ancestors 'none'");
    server_.sendHeader("Referrer-Policy", "no-referrer");
    server_.sendHeader("Connection", "close");
  }
  void reply(int code, const char* message) {
    responseHeaders();
    server_.send(code, "text/plain; charset=utf-8", message);
  }
  void showForm() {
    if (!localRequest()) { reply(403, "Open the setup IP address while joined to the radio's setup Wi-Fi."); return; }
    String page;
    if (!page.reserve(2300)) { reply(503, "Setup memory unavailable; retry."); return; }
    char escapedSsid[32 * 6 + 1];
    if (!Detail::escapeHtml(current_.ssid, escapedSsid, sizeof(escapedSsid))) { reply(503, "Setup memory unavailable; retry."); return; }
    page = F("<!doctype html><html lang='en'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>GTS Radio Wi-Fi</title><style>body{font:18px system-ui;max-width:30rem;margin:2rem auto;padding:1rem}label,input,button{display:block;margin:.8rem 0}input{box-sizing:border-box;width:100%;padding:.6rem}button{padding:.7rem 1rem}</style><h1>GTS Radio Wi-Fi</h1><p>Enter your 2.4 GHz Wi-Fi network. Setup stays available if the connection fails.</p><form method='post' action='/save' enctype='application/x-www-form-urlencoded'><label>Network name (SSID)<input name='ssid' maxlength='32' required autocomplete='off' value='");
    page += escapedSsid;
    page += F("'></label><label>Password<input name='password' type='password' maxlength='64' autocomplete='new-password'></label><p>Leave password empty for an open network. Otherwise use 8–63 characters, or a 64-digit hexadecimal key.</p><input type='hidden' name='csrf' value='");
    page += token_;
    page += F("'><button type='submit'>Save and connect</button></form><p>Check the radio display for connection status. Its setup access point closes 20 seconds after a successful connection.</p></html>");
    responseHeaders();
    server_.sendHeader("Set-Cookie", String("gts_session=") + token_ + "; Path=/; HttpOnly; SameSite=Strict");
    server_.send(200, "text/html; charset=utf-8", page);
  }
  bool sessionMatches() {
    const String cookie = server_.header("Cookie");
    return Detail::cookieMatches(cookie.c_str(), cookie.length(), token_);
  }
  void handleSave() {
    if (!localRequest() || !sessionMatches()) { reply(403, "Setup session expired. Reload the setup page."); return; }
    const String origin = server_.header("Origin");
    if (origin.length() && origin != String("http://") + portalAddress() &&
        origin != String("http://") + portalAddress() + ":80") {
      reply(403, "Setup request origin does not match."); return;
    }
    if (!bodyReady_ || !bodyLength_ || bodyLength_ > kMaxBody ||
        server_.clientContentLength() != static_cast<int>(bodyLength_)) {
      reply(413, "Use the setup form; request is too large or unsupported."); return;
    }
    Detail::FormFields fields;
    if (!Detail::parseForm(body_, bodyLength_, fields)) { reply(400, "Invalid, duplicated, or oversized setup field."); return; }
    if (!Detail::tokenMatches(token_, fields.csrf, strlen(fields.csrf))) {
      reply(403, "Setup session expired. Reload the setup page."); return;
    }
    if (!copyCredentials(stagedCredentials_, fields.ssid, fields.password)) {
      reply(400, "SSID must be 1–32 bytes. Password must be empty, 8–63 characters, or 64 hex digits."); return;
    }
    staged_ = true;
    bodyReady_ = false;
    memset(body_, 0, sizeof(body_));
    bodyLength_ = 0;
    reply(202, "Settings accepted. Check the radio display for connection status. Setup stays available if the connection fails.");
  }

  WebServer server_{80};
  RadioStorage::WifiCredentials current_{}, stagedCredentials_{};
  bool begun_ = false, portal_ = false, online_ = false, attempting_ = false;
  bool portalRequested_ = false;
  bool haveCredentials_ = false, staged_ = false, credentialsReady_ = false;
  bool closeAfterOnline_ = false, bodyReady_ = false;
  uint32_t attemptFrom_ = 0, retryFrom_ = 0, retryDelayMs_ = 0;
  uint32_t backoffMs_ = kFirstBackoffMs, connectedAt_ = 0, portalRetryFrom_ = 0;
  char apName_[20] = {}, apPassword_[9] = {}, token_[33] = {};
  char status_[80] = "Wi-Fi starting";
  char body_[kMaxBody + 1] = {};
  std::size_t bodyLength_ = 0;
};

}  // namespace RadioWifi

#endif  // RADIO_WIFI_HOST_TEST
