// Run: c++ -std=c++11 -Wall -Wextra -Werror tests/wifi_portal_test.cpp -o /tmp/radio-wifi-test && /tmp/radio-wifi-test
#define RADIO_WIFI_HOST_TEST
#include "../WifiPortal.h"

#include <cassert>
#include <iostream>
#include <string>

using namespace RadioWifi::Detail;
static const char Token[] = "0123456789abcdef0123456789abcdef";

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
  testFormDecodingAndBounds();
  testHtmlAttributeEscaping();
  testSessionAndCsrf();
  std::cout << "wifi portal tests passed\n";
}
