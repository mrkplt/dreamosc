// Unit tests for the streaming Source and the per-head WindowCache
// (stretch_core.h): the ring that lets a head render out of material it
// asked for ahead of need, with a memory Source standing in for the card.
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it.

#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"

namespace {

// A Source that counts every read() and can be told to fail a range.
struct CountingSource : Source {
  std::vector<float> s;
  uint32_t reads = 0;
  uint64_t samples = 0;
  int64_t failFrom = -1, failTo = -1;      // read() fails if it touches [failFrom, failTo)
  explicit CountingSource(uint32_t n, uint32_t r = 48000) : s(n) {
    for (uint32_t i = 0; i < n; i++) s[i] = (float)i;   // sample value == index
    len = n; rate = r;
  }
  bool read(uint32_t first, uint32_t n, float* dst) override {
    reads++; samples += n;
    if (failFrom >= 0 && (int64_t)first < failTo && (int64_t)(first + n) > failFrom) return false;
    for (uint32_t i = 0; i < n; i++) dst[i] = s[first + i];
    return true;
  }
};

// Expected value at unwrapped position p (the source is the identity).
float at(const CountingSource& src, int64_t p) {
  int64_t L = src.len, m = p % L; if (m < 0) m += L;
  return (float)m;
}

void check_window(const WindowCache& c, const CountingSource& src, int64_t s, uint32_t n) {
  REQUIRE(c.resident(s, n));
  std::vector<float> win(n, 1.0f), out(n);
  for (uint32_t i = 0; i < n; i++) win[i] = 0.5f + (float)i / (float)n;   // a real window
  c.copyWindowed(s, n, win.data(), out.data());
  for (uint32_t i = 0; i < n; i++) {
    float want = at(src, s + (int64_t)i) * win[i];             // the exact same multiply
    if (out[i] != want) { FAIL("mismatch at " << s << "+" << i << ": " << out[i] << " vs " << want); }
  }
}

}  // namespace

TEST_CASE("Source::fetch wraps at the material length, splitting at the seam, and at negative starts") {
  CountingSource src(1000);
  std::vector<float> out(2500);
  REQUIRE(src.fetch(900, 200, out.data()));                    // crosses the end once
  REQUIRE(src.reads == 2);
  for (int i = 0; i < 200; i++) REQUIRE(out[i] == at(src, 900 + i));
  src.reads = 0;
  REQUIRE(src.fetch(-150, 300, out.data()));                   // starts before 0
  REQUIRE(out[0] == 850.0f); REQUIRE(out[150] == 0.0f); REQUIRE(out[299] == 149.0f);
  src.reads = 0;
  REQUIRE(src.fetch(0, 2500, out.data()));                     // more than twice the material
  REQUIRE(src.reads == 3);
  for (int i = 0; i < 2500; i++) REQUIRE(out[i] == at(src, i));
  // A failed read zero-fills its piece and reports false; the rest still lands.
  src.failFrom = 0; src.failTo = 100;
  REQUIRE_FALSE(src.fetch(950, 100, out.data()));
  REQUIRE(out[0] == 950.0f); REQUIRE(out[49] == 999.0f); REQUIRE(out[50] == 0.0f);
}

TEST_CASE("WindowCache: a miss fetches back-pad + window + two windows of look-ahead; hits cost nothing") {
  CountingSource src(100000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 4096;
  REQUIRE(c.ensure(10000, w, src));
  REQUIRE(c.misses() == 1);
  REQUIRE(c.base() == 10000 - (int64_t)w / 2);
  REQUIRE(c.count() == w / 2 + w + 2 * w);
  REQUIRE(src.samples == c.count());
  check_window(c, src, 10000, w);
  uint32_t reads = src.reads;
  // The next hop (half a window on) and the one after are hits: no reads.
  REQUIRE(c.ensure(10000 + w / 2, w, src));
  REQUIRE(c.ensure(10000 + w, w, src));
  REQUIRE(src.reads == reads);
  REQUIRE(c.misses() == 1); REQUIRE(c.extends() == 0);
  check_window(c, src, 10000 + w, w);
  // The previous frame's -hop position is resident too (the back-pad).
  REQUIRE(c.resident(10000 - (int64_t)w / 2, w));
}

TEST_CASE("WindowCache: a head travelling forward extends by fetching only the new tail") {
  CountingSource src(1000000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 2048;
  REQUIRE(c.ensure(5000, w, src));
  uint64_t before = src.samples;
  // Walk a hop at a time for many windows: never a miss after the first,
  // and the total fetched is about the distance travelled plus the initial run.
  int64_t s = 5000;
  for (int hop = 1; hop <= 200; hop++) {
    s += w / 2;
    REQUIRE(c.ensure(s, w, src));
    check_window(c, src, s, w);
    REQUIRE(c.misses() == 1);
  }
  uint64_t travelled = 200 * (w / 2);
  REQUIRE(src.samples - before <= travelled + 3 * w);   // no re-fetch of kept material
  REQUIRE(c.extends() > 0);
  // Every extend refilled at least a window (not sample-by-sample thrash).
  REQUIRE(c.extends() <= travelled / w + 1);
}

TEST_CASE("WindowCache: a scrub is a miss; a small step back is served by the back-pad; stepping past the run re-targets") {
  CountingSource src(1000000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 4096;
  REQUIRE(c.ensure(50000, w, src));
  REQUIRE(c.ensure(50000 - (int64_t)w / 2, w, src));     // back one hop: in the pad
  REQUIRE(c.misses() == 1);
  check_window(c, src, 50000 - (int64_t)w / 2, w);
  REQUIRE(c.ensure(500000, w, src));                       // the position knob: far away
  REQUIRE(c.misses() == 2);
  check_window(c, src, 500000, w);
  // A step back further than the pad but overlapping the run keeps the
  // overlap and prepends the rest (not a fresh fill).
  uint64_t before = src.samples;
  REQUIRE(c.ensure(500000 - (int64_t)w, w, src));
  check_window(c, src, 500000 - (int64_t)w, w);
  REQUIRE(src.samples - before < (uint64_t)(w / 2 + w + 2 * w));
}

TEST_CASE("WindowCache: runs that cross the material's end and negative positions read correctly") {
  CountingSource src(30000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 8192;
  REQUIRE(c.ensure(29000, w, src));                    // window straddles the wrap
  check_window(c, src, 29000, w);
  REQUIRE(c.ensure(-4096, w, src));                    // the -hop frame of a head at 0
  check_window(c, src, -4096, w);
  // Keep walking through several wraps of a short source.
  int64_t s = -4096;
  for (int hop = 0; hop < 40; hop++) {
    s += w / 2;
    REQUIRE(c.ensure(s, w, src));
    check_window(c, src, s, w);
  }
}

TEST_CASE("WindowCache: the ring wraps correctly at every window size across a long walk") {
  CountingSource src(4000000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  for (int w = SS_W_MIN; w <= SS_W; w *= 2) {
    WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
    int64_t s = 12345;
    REQUIRE(c.ensure(s, (uint32_t)w, src));
    for (int hop = 0; hop < 64; hop++) {
      s += w / 2;
      REQUIRE(c.ensure(s, (uint32_t)w, src));
      check_window(c, src, s, (uint32_t)w);
    }
    REQUIRE(c.misses() == 1);
  }
}

TEST_CASE("WindowCache: a failed read is zero-filled and counted; the cache stays consistent") {
  CountingSource src(100000);
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 1024;
  src.failFrom = 20000; src.failTo = 20100;
  // The run [18488, 22072) is one read() (no wrap), and that read fails: the
  // WHOLE piece the Source refused is zero (a card error is not partial
  // material), the run is still resident, and the failure is counted.
  REQUIRE_FALSE(c.ensure(19000, w, src));
  REQUIRE(c.failures() == 1);
  REQUIRE(c.resident(19000, w));
  std::vector<float> win(w, 1.0f), out(w);
  c.copyWindowed(19000, w, win.data(), out.data());
  for (uint32_t i = 0; i < w; i++) REQUIRE(out[i] == 0.0f);
  // A later extend that the Source serves lands normally next to the zeros.
  src.failFrom = -1;
  REQUIRE(c.ensure(21500, w, src));                   // within the run, look-ahead short: extend
  REQUIRE(c.failures() == 1);
  c.copyWindowed(22072, w, win.data(), out.data());
  REQUIRE(out[0] == 22072.0f);
  REQUIRE(out[w - 1] == 22072.0f + (w - 1));
  // Once re-targeted away and back, the material is read again.
  REQUIRE(c.ensure(60000, w, src));
  REQUIRE(c.ensure(19000, w, src));
  check_window(c, src, 19000, w);
}

TEST_CASE("A rendered head through the cache is bit-identical to the in-memory source it replaces") {
  // The cache's copyWindowed must be the very multiply fillWindowed did: the
  // determinism test is the guard within a build, and this pins the two
  // paths against each other -- MemSource through the cache vs a direct
  // windowed copy of the same run.
  auto srcbuf = testutil::make_source(2.0f, 48000);
  MemSource src{srcbuf.data(), (uint32_t)srcbuf.size()};
  std::vector<float> ring(SS_CACHE_FLOATS);
  WindowCache c; c.setBuffer(ring.data(), SS_CACHE_FLOATS);
  const uint32_t w = 4096;
  const float* win = &gWindows[ssWinOff(ssSizeIdx((int)w))];
  gTab.init();
  int64_t s = 48000 * 1.5;
  REQUIRE(c.ensure(s, w, src));
  std::vector<float> a(w), b(w);
  c.copyWindowed(s, w, win, a.data());
  for (uint32_t i = 0; i < w; i++) b[i] = srcbuf[(size_t)(s + i) % srcbuf.size()] * win[i];
  REQUIRE(a == b);
}
