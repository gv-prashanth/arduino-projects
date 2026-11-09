// Run: c++ -std=c++11 -Wall -Wextra -Werror tests/wifi_portal_test.cpp -o /tmp/radio-wifi-test && /tmp/radio-wifi-test
#define RADIO_WIFI_HOST_TEST
#include "../WifiPortal.h"

#include <cassert>
#include <iostream>
#include <string>

using namespace RadioWifi::Detail;
static const char Token[] = "0123456789abcdef0123456789abcdef";

static void testStreamingModeAcrossTransitions() {
  struct Wifi {
    bool modeReady = true, sleepAccepted = true, ignoreSleepRequest = false;
    unsigned sleep = 1, modes = 0, sleepRequests = 0, preferenceReads = 0;
    int selectedMode = 0;
    std::string calls;
    bool mode(int selected) {
      calls += 'm'; ++modes; selectedMode = selected;
      sleep = 1; // A mode transition may restore the platform default.
      return modeReady;
    }
    bool setSleep(bool enabled) {
      calls += 's'; ++sleepRequests;
      assert(!enabled);
      if (sleepAccepted && !ignoreSleepRequest) sleep = enabled;
      return sleepAccepted;
    }
    unsigned getSleep() { calls += 'g'; ++preferenceReads; return sleep; }
  } wifi;
  // Startup, reconnect, opening AP+STA, and closing the portal all retain the
  // radio's latency policy, even when mode() resets the fake's preference.
  const int modes[] = {1, 1, 3, 1};
  for (int mode : modes) {
    wifi.calls.clear();
    const StreamingModeResult result = streamingMode(wifi, mode);
    assert(result.ready && result.sleepDisabled && wifi.sleep == 0);
    assert(wifi.selectedMode == mode && wifi.calls == "msg");
  }
  assert(wifi.modes == 4 && wifi.sleepRequests == 4 && wifi.preferenceReads == 4);

  wifi.modeReady = false; wifi.calls.clear();
  StreamingModeResult result = streamingMode(wifi, 1);
  assert(!result.ready && !result.sleepDisabled && wifi.calls == "m");

  wifi.modeReady = true; wifi.sleepAccepted = false; wifi.calls.clear();
  result = streamingMode(wifi, 1);
  // A failed latency preference must not turn a valid connection mode into a
  // Wi-Fi failure; the caller records it independently instead.
  assert(result.ready && !result.sleepDisabled && wifi.calls == "ms");

  wifi.sleepAccepted = true; wifi.ignoreSleepRequest = true; wifi.calls.clear();
  result = streamingMode(wifi, 3);
  assert(result.ready && !result.sleepDisabled && wifi.calls == "msg");

  wifi.ignoreSleepRequest = false;
  result = streamingMode(wifi, 1);
  assert(result.ready && result.sleepDisabled);
}

static bool parse(const std::string& body, FormFields& fields) {
  return parseForm(body.data(), body.size(), fields);
}
static std::string form(const std::string& ssid, const std::string& password) {
  return "ssid=" + ssid + "&password=" + password + "&csrf=" + Token;
}

static void testFormDecodingAndBounds() {
  FormFields fields;
  assert(parse(form("House+%26+Radio%27s", "spaces+and%2Bplus"), fields));
  assert(std::string(fields.ssid) == "House & Radio's");
  assert(std::string(fields.password) == "spaces and+plus");
  assert(tokenMatches(Token, fields.csrf, std::strlen(fields.csrf)));
  assert(parse(form("Guest", ""), fields));  // Open networks keep an explicit empty password.
  assert(!fields.password[0]);
  assert(parse(form(std::string(32, 'S'), std::string(64, 'a')), fields));
  assert(!parse(form(std::string(33, 'S'), "password"), fields));
  assert(!parse(form("Home", std::string(65, 'a')), fields));
  assert(!parse(form("Home%00Other", "password"), fields));
  assert(!parse(form("Home%", "password"), fields));
  assert(!parse(form("Home%2", "password"), fields));
  assert(!parse(form("Home%GG", "password"), fields));
  assert(!parse(form("Home", "password") + "&ssid=Other", fields));
  assert(!parse(form("Home", "password") + "&unexpected=x", fields));
  assert(!parse(form("Home", "password") + "&", fields));
  assert(!parse("ssid=Home&csrf=" + std::string(Token), fields));
  assert(!parse(std::string(513, 'x'), fields));
  std::string embeddedNull = form("Home", "password");
  embeddedNull[6] = '\0';
  assert(!parse(embeddedNull, fields));
  // UTF-8 network names count bytes, and decoding preserves those bytes.
  assert(parse(form("Caf%C3%A9", "password"), fields));
  assert(std::string(fields.ssid) == "Caf\xC3\xA9");
}

static void testHtmlAttributeEscaping() {
  char escaped[193];
  assert(escapeHtml("Radio's \"Guest\" <&>", escaped, sizeof(escaped)));
  assert(std::string(escaped) == "Radio&#39;s &quot;Guest&quot; &lt;&amp;&gt;");
  // The actual form surrounds this helper's output with plain single quotes.
  const std::string attribute = "value='" + std::string(escaped) + "'";
  assert(attribute.find("value='Radio&#39;s") == 0);
  assert(attribute.find('\\') == std::string::npos);
  assert(escapeHtml(std::string(32, '"').c_str(), escaped, sizeof(escaped)));
  assert(std::strlen(escaped) == 192);
  char tooSmall[6] = {};
  assert(!escapeHtml("\"", tooSmall, sizeof(tooSmall)) && !tooSmall[0]);
}

static void testSessionAndCsrf() {
  assert(tokenMatches(Token, Token, 32));
  assert(!tokenMatches(Token, "1123456789abcdef0123456789abcdef", 32));
  assert(!tokenMatches(Token, Token, 31));
  const std::string cookie = "other=1; gts_session=" + std::string(Token) + "; another=2";
  assert(cookieMatches(cookie.data(), cookie.size(), Token));
  const std::string whitespace = "\t gts_session=" + std::string(Token) + " \t";
  assert(cookieMatches(whitespace.data(), whitespace.size(), Token));
  const std::string prefixed = "fake_gts_session=" + std::string(Token);
  assert(!cookieMatches(prefixed.data(), prefixed.size(), Token));
  const std::string suffixed = "gts_session=" + std::string(Token) + "x";
  assert(!cookieMatches(suffixed.data(), suffixed.size(), Token));
  assert(!cookieMatches(cookie.data(), cookie.size(), "1123456789abcdef0123456789abcdef"));
  const std::string oversized(257, 'x');
  assert(!cookieMatches(oversized.data(), oversized.size(), Token));
}

int main() {
  testStreamingModeAcrossTransitions();
  testFormDecodingAndBounds();
  testHtmlAttributeEscaping();
  testSessionAndCsrf();
  std::cout << "wifi portal tests passed\n";
}
