#include "Arduino.h"
#include "RadioLib.h"
uint32_t g_now = 1000; SerialMock Serial; std::vector<TxRec> g_tx; std::vector<uint8_t> g_rx_pkt;
int g_cad_busy_n = 0; int g_rx_rssi = -70; bool g_mtx_locked = false;
#include "lora.cpp"
#include <cassert>
#include <map>

static int g_fail = 0, g_pass = 0;
#define CHECK(c) do { if (c) g_pass++; else { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

struct Msg { std::string text, src; bool partial; int hops; };
static std::vector<Msg> g_msgs;
static std::vector<std::pair<std::string, std::string>> g_l1;
static void on_msg(const String& t, const LoraRxInfo& i) { g_msgs.push_back({t.s, i.src, i.partial, i.hops}); }
static void on_conn(bool) {}
static void on_l1(const String& t, const String& a, const LoraRxInfo&) { g_l1.push_back({t.s, a.s}); }

static void inject_raw(const std::string& raw, int rssi = -70) {
  g_rx_pkt.assign(raw.begin(), raw.end()); g_rx_rssi = rssi; s_rx_flag = true; lora_tick();
}
static uint32_t g_pid = 5000;
static void inject(const char* src, int ttl, const std::string& line, int rssi = -70) {
  inject_raw("R|" + std::string(src) + "|" + std::to_string(g_pid++) + "|" + std::to_string(ttl) + "|" + line, rssi);
}
static std::string payload_of(const std::string& w) {   // after 4th '|'
  size_t p = 0; for (int i = 0; i < 4; i++) p = w.find('|', p) + 1; return w.substr(p);
}
static int count_tx(const char* prefix) { int n = 0; for (auto& t : g_tx) if (payload_of(t.line).rfind(prefix, 0) == 0) n++; return n; }
static LoraNode* find_node(const char* id) { for (auto& n : s_nodes) if (n.valid && !strcmp(n.id, id)) return &n; return nullptr; }

int main() {
  lora_begin();
  lora_set_callbacks(on_msg, on_conn);
  lora_set_l1_callback(on_l1);
  lora_set_my_id("CAR01");

  printf("[1] TX chunking / !! escape budget / round trip\n");
  {
    std::string body = "!시작부터 느낌표. ";
    while (body.size() < 59) body += 'a';     // first chunk is escaped → must be cut at exactly 59
    body += "!둘째 청크도 느낌표로 시작한다. 한글 UTF-8 경계가 청크 중간에서 잘리면 안 된다 가나다라마바사아자차카타파하";
    g_tx.clear();
    do_send_blocking(body.c_str());
    CHECK(payload_of(g_tx.front().line) == "[SOF]");
    CHECK(payload_of(g_tx.back().line) == "[EOF]");
    bool all_le_60 = true, consecutive = true, ttl3 = true; uint32_t prev = 0;
    for (size_t i = 0; i < g_tx.size(); i++) {
      String src, orig; uint32_t pk; uint8_t ttl;
      String w(g_tx[i].line);
      CHECK(relay_parse(w, src, pk, ttl, orig));
      if (orig.length() > 60) all_le_60 = false;
      if (i && pk != prev + 1) consecutive = false;
      if (ttl != 3) ttl3 = false;
      prev = pk;
      CHECK(src == "P01");
    }
    CHECK(all_le_60); CHECK(consecutive); CHECK(ttl3);
    CHECK(payload_of(g_tx[1].line).rfind("!!", 0) == 0);
    // pacing: gap between consecutive packet starts ≥ ToA(250 mock) + 2×ToA
    CHECK(g_tx[2].at - g_tx[1].at >= 250 + 300);
    // round trip through our own receiver as if TFF had sent it
    g_msgs.clear();
    for (auto& t : g_tx) inject("TFF", 3, payload_of(t.line));
    CHECK(g_msgs.size() == 1);
    CHECK(g_msgs.size() == 1 && g_msgs[0].text == body);
    CHECK(g_msgs.size() == 1 && !g_msgs[0].partial && g_msgs[0].src == "TFF" && g_msgs[0].hops == 0);
  }

  printf("[2] per-src reassembly: interleaved senders, L0 from a third node\n");
  {
    g_msgs.clear();
    inject("TFF", 3, "[SOF]");
    inject("TFF", 3, "hel");
    inject("P00", 2, "[SOF]");
    inject("P00", 2, "xx");
    inject("F00", 1, "HB\tfan\trssi=-60");         // must not land in anyone's message
    inject("TFF", 3, "PING me maybe");             // bare line inside TFF's open frame = user text
    inject("TFF", 3, "lo");
    inject("TFF", 3, "[EOF]");
    inject("P00", 2, "[EOF]");
    CHECK(g_msgs.size() == 2);
    CHECK(g_msgs.size() == 2 && g_msgs[0].text == "helPING me maybelo" && g_msgs[0].src == "TFF");
    CHECK(g_msgs.size() == 2 && g_msgs[1].text == "xx" && g_msgs[1].src == "P00" && g_msgs[1].hops == 1);
    LoraNode* f = find_node("F00");
    CHECK(f && f->name == "fan" && f->peer_rssi_valid && f->peer_rssi == -60 && f->hops == 0 && f->rssi_valid);
    LoraNode* p = find_node("P00");
    CHECK(p && !p->rssi_valid && p->hops == 1);    // relayed → relay's RSSI is not P00's
  }

  printf("[3] L1 mid-frame stays out of chat; !! unescape; unknown L1 silent\n");
  {
    g_msgs.clear(); g_l1.clear();
    inject("TFF", 3, "[SOF]");
    inject("TFF", 3, "a");
    inject("TFF", 3, "!CAR\tP03\tP\t21.5\t40.0\t1013\t-\t100\t2");
    inject("TFF", 3, "!!bang");
    inject("TFF", 3, "!ZZ\twhatever");
    inject("TFF", 3, "[EOF]");
    CHECK(g_msgs.size() == 1 && g_msgs[0].text == "a!bang");
    CHECK(g_l1.size() == 2 && g_l1[0].first == "CAR" && g_l1[1].first == "ZZ");
    LoraNode* t = find_node("TFF");
    CHECK(t && t->info_type == "CAR" && t->info.startsWith("P03\tP\t21.5"));
  }

  printf("[4] idle-timeout → delivered marked partial; L1 does not reset the timer\n");
  {
    g_msgs.clear();
    inject("TFF", 3, "[SOF]");
    inject("TFF", 3, "cut off");
    g_now += 15000; inject("TFF", 3, "!ZZ\tx");    // L1 at +15 s must NOT renew
    g_now += 6000;  lora_tick();
    CHECK(g_msgs.size() == 1 && g_msgs[0].partial && g_msgs[0].text == "cut off");
    // new SOF while open → previous delivered partial
    g_msgs.clear();
    inject("TFF", 3, "[SOF]"); inject("TFF", 3, "one");
    inject("TFF", 3, "[SOF]"); inject("TFF", 3, "two"); inject("TFF", 3, "[EOF]");
    CHECK(g_msgs.size() == 2 && g_msgs[0].partial && g_msgs[0].text == "one" && !g_msgs[1].partial && g_msgs[1].text == "two");
  }

  printf("[5] envelope grammar / dedup / own echo / terminator strip\n");
  {
    g_msgs.clear();
    LoraStats a; lora_get_stats(&a);
    inject_raw("R|TFF|9001|83|[SOF]");            // corrupted ttl
    inject_raw("R|tff|9002|3|[SOF]");             // bad src
    inject_raw("garbage without envelope");
    inject_raw("R|TFF|9003|0|[SOF]");             // ttl 0 never exists on air
    LoraStats b; lora_get_stats(&b);
    CHECK(b.rx_bad - a.rx_bad == 4);
    inject_raw("R|P01|9004|3|[SOF]");             // our own id
    lora_get_stats(&a); CHECK(a.rx_own - b.rx_own == 1);
    inject_raw("R|TFF|9010|3|[SOF]\n");           // T-Deck appends \n (§4 v1.16)
    inject_raw("R|TFF|9011|3|dup\r\n");
    inject_raw("R|TFF|9011|2|dup");               // relayed copy of the same packet
    inject_raw(std::string("R|TFF|9012|3|[EOF]") + '\0');
    lora_get_stats(&b); CHECK(b.rx_dup - a.rx_dup == 1);
    CHECK(g_msgs.size() == 1 && g_msgs[0].text == "dup" && !g_msgs[0].partial);
  }

  printf("[6] Range: broadcast PING silent, addressed PING → PONG after 4×ToA_rx\n");
  {
    g_tx.clear();
    s_next_hb_ms = g_now + 600000;                // keep HB out of the way
    inject("TFF", 3, "PING\t7\tTFF");
    for (int i = 0; i < 50; i++) { g_now += 100; lora_tick(); }
    CHECK(count_tx("PONG") == 0);
    uint32_t t0 = g_now;
    std::string ping = "PING\t8\tTFF\tP01";
    inject("TFF", 3, ping);
    uint32_t hold = 4 * toa_ms(("R|TFF|" + std::to_string(g_pid - 1) + "|3|" + ping).size());
    while (count_tx("PONG") == 0 && g_now - t0 < 10000) { g_now += 10; lora_tick(); }
    CHECK(count_tx("PONG") == 1);
    CHECK(g_tx.back().at - t0 >= hold);
    CHECK(payload_of(g_tx.back().line) == "PONG\t8\tCAR01");
    int seq; uint32_t cnt; CHECK(lora_consume_range(&seq, &cnt) && seq == 8);
  }

  printf("[7] §8a beacon deferral: yields to a stream, capped at one period; replies never yield\n");
  {
    g_tx.clear();
    inject("P10", 3, "!GR\tart1\tK\tabc");        // n = base36 K = 20 chunks
    uint32_t reserved_for = s_chan_reserved_until - g_now;
    CHECK(reserved_for > 20000);                  // 20 × ~525 ms × (1 + 1.3×3)
    CHECK(lora_send_l1("!CAR\tP01\tP\t-\t-\t-\t-\t1\t0", 3, 8000));   // beacon-class, period 8 s
    CHECK(lora_send_l1("!ZQ\treq", 3, 0));                            // request: must not wait
    lora_tick();
    CHECK(count_tx("!ZQ") == 1 && count_tx("!CAR") == 0);             // request jumped the yielding beacon
    uint32_t t0 = g_now;
    while (count_tx("!CAR") == 0 && g_now - t0 < 60000) { g_now += 50; lora_tick(); }
    CHECK(count_tx("!CAR") == 1);
    CHECK(g_now - t0 >= 7900 && g_now - t0 <= 8500);                  // sent at its deadline, not at reservation end
    // HB: due now, channel still reserved → waits, but not beyond one extra period
    s_chan_reserved_until = g_now + 500000; s_next_hb_ms = g_now; t0 = g_now;
    while (count_tx("HB") == 0 && g_now - t0 < 200000) { g_now += 100; lora_tick(); }
    CHECK(count_tx("HB") == 1 && g_now - t0 >= 60000 && g_now - t0 < 61000);
    s_chan_reserved_until = 0;
    CHECK(!lora_send_l1("no bang", 3, 0));
    CHECK(!lora_send_l1("!!escaped", 3, 0));
    CHECK(!lora_send_l1("!" + String(std::string(70, 'x')), 3, 0));
  }

  printf("[8] LBT: busy CAD defers, then sends\n");
  {
    g_tx.clear(); LoraStats a; lora_get_stats(&a);
    g_cad_busy_n = 3;
    CHECK(lora_send_l1("!ZQ\tlbt", 3, 0));
    lora_tick(); CHECK(count_tx("!ZQ") == 0);
    uint32_t t0 = g_now;
    while (count_tx("!ZQ") == 0 && g_now - t0 < 5000) { g_now += 5; lora_tick(); }
    LoraStats b; lora_get_stats(&b);
    CHECK(count_tx("!ZQ") == 1 && b.lbt_defers - a.lbt_defers == 3);
  }

  printf("[9] binary voice chunk (0xC2) renews reservation, never parsed\n");
  {
    s_chan_reserved_until = 0;
    std::string v; v += (char)0xC2; v += std::string("\x01\x03\x00\x00zz", 6);
    LoraStats a; lora_get_stats(&a);
    inject_raw(v);
    LoraStats b; lora_get_stats(&b);
    CHECK(b.rx_bad == a.rx_bad && (int32_t)(s_chan_reserved_until - g_now) > 0);
  }

  printf("[10] orphan chunk (SOF lost) is delivered marked, !RB is direct by design\n");
  {
    g_msgs.clear();
    inject("T01", 3, "stray words");
    CHECK(g_msgs.size() == 1 && g_msgs[0].partial);
    inject("P10", 1, "!RB\tNBA\t0a\t-\t-\t-\t-\t3\t301", -88);
    LoraNode* r = find_node("P10");
    CHECK(r && r->hops == 0 && r->rssi == -88 && r->info_type == "RB");
    CHECK(lora_nodes_count() >= 5);
  }

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
