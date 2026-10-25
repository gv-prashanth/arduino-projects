#include "../CatalogState.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <type_traits>

using RadioCatalog::Filter;
using RadioCatalog::Request;
using RadioCatalog::Transaction;

static Filter country(std::uint8_t index) {
  Filter filter;
  filter.country = index;
  return filter;
}

static void sameRequest(const Request& actual, const Request& expected) {
  assert(actual.generation == expected.generation);
  assert(actual.filter == expected.filter);
  assert(actual.offset == expected.offset);
  assert(actual.refresh == expected.refresh);
}

static void testFiltersAndInitialState() {
  RadioModel::Settings settings;
  settings.volume = 11;
  settings.muted = true;
  settings.country = 2;
  settings.genre = 5;
  settings.codec = 1;
  settings.bitrateCap = 96;
  const Filter filter = RadioCatalog::filterFor(settings);
  assert(filter.country == 2 && filter.genre == 5 && filter.codec == 1 && filter.bitrateCap == 96);
  Filter changed = filter;
  assert(changed == filter);
  ++changed.country; assert(!(changed == filter)); changed = filter;
  ++changed.genre; assert(!(changed == filter)); changed = filter;
  ++changed.codec; assert(!(changed == filter)); changed = filter;
  ++changed.bitrateCap; assert(!(changed == filter));

  settings.source = RadioModel::Source::Akashvani;
  assert(RadioModel::copyText(settings.airLocation, "Karnataka"));
  const Filter air = RadioCatalog::filterFor(settings);
  assert(air.source == RadioModel::Source::Akashvani && !air.country && !air.genre &&
         !air.codec && !air.bitrateCap && std::strcmp(air.location, "Karnataka") == 0);
  assert(!(filter == air));
  changed = air;
  assert(RadioModel::copyText(changed.location, "Tamil Nadu"));
  assert(!(changed == air));
  settings.source = RadioModel::Source::RadioBrowser;
  assert(RadioCatalog::filterFor(settings) == filter);

  Transaction state;
  assert(!state.pending && !state.failed && !state.failures && !state.generation);
  assert(!state.accepts(0) && !state.succeed(0) && !state.fail(0, 0));
  assert(!state.expire(100000) && !state.retryDue(true, 100000));
  assert(!state.shouldApply(filter, filter, true));
  assert(state.shouldApply(filter, changed, true));
  assert(state.shouldApply(filter, filter, false));
}

static void testOutOfOrderResults() {
  Transaction state;
  const Request first = state.begin(country(0), 0, 10);
  const Request latest = state.begin(country(2), 20, 25);
  assert(first.generation == 1 && latest.generation == 2);
  assert(state.pending && !state.failed && state.startedAt == 25);
  assert(!state.accepts(first.generation));
  assert(!state.succeed(first.generation));
  assert(!state.fail(first.generation, 30));
  sameRequest(state.request, latest);
  assert(state.pending && !state.failed && !state.failures && state.startedAt == 25);
  assert(state.accepts(latest.generation) && state.succeed(latest.generation));
  assert(!state.pending && !state.failed && !state.failures);
  assert(!state.accepts(latest.generation) && !state.succeed(latest.generation));
}

static void testReapplyCommittedFilterSupersedesPending() {
  const Filter committed = country(0);
  const Filter india = country(2);
  Transaction state;
  const Request pendingIndia = state.begin(india, 0, 100);
  // Reapplying All must cancel the pending IN result even though All is committed.
  assert(state.shouldApply(committed, committed, true));
  const Request reappliedAll = state.begin(committed, 0, 101);
  assert(reappliedAll.generation != pendingIndia.generation);
  assert(!state.succeed(pendingIndia.generation));
  assert(state.request.filter == committed && state.pending);
  assert(state.succeed(reappliedAll.generation));
  assert(!state.shouldApply(committed, committed, true));

  const Request failedIndia = state.begin(india, 40, 200);
  assert(state.fail(failedIndia.generation, 201));
  assert(state.shouldApply(committed, committed, true));
  const Request recovery = state.begin(committed, 0, 202);
  assert(!state.fail(failedIndia.generation, 203));
  sameRequest(state.request, recovery);
}

static void testRetryPreservesLatestRequestAndResetsOnSuccess() {
  Transaction state;
  Request latest = state.begin(country(2), 40, 1000);
  assert(state.fail(latest.generation, 1005));
  sameRequest(state.request, latest);
  assert(!state.pending && state.failed && state.failures == 1 && state.retryAt == 11005);
  assert(!state.fail(latest.generation, 1006)); // Duplicate completion must not inflate backoff.
  assert(!state.retryDue(true, 11004));
  assert(state.retryDue(true, 11005));
  assert(!state.retryDue(false, 11005) && !state.retryDue(false, 90000));

  std::uint32_t now = state.retryAt;
  for (int attempt = 2; attempt <= 6; ++attempt) {
    const Request previous = latest;
    latest = state.begin(state.request.filter, state.request.offset, now);
    assert(latest.filter == previous.filter && latest.offset == previous.offset);
    assert(latest.generation != previous.generation);
    assert(state.pending && !state.failed && !state.retryDue(true, now + 100000));
    assert(state.fail(latest.generation, now));
    const unsigned failures = attempt < 4 ? attempt : 4;
    assert(state.failures == failures);
    assert(state.retryAt == now + (5000UL << failures));
    sameRequest(state.request, latest);
    now = state.retryAt;
  }
  latest = state.begin(state.request.filter, state.request.offset, now);
  assert(state.succeed(latest.generation));
  assert(!state.pending && !state.failed && state.failures == 0);
  assert(!state.retryDue(true, now + 100000));
  latest = state.begin(country(0), 0, now + 1);
  assert(state.fail(latest.generation, now + 2));
  assert(state.failures == 1 && state.retryAt == now + 10002);
}

static void testTimeoutAndClockRollover() {
  Transaction state;
  const Request request = state.begin(country(2), 60, 500);
  assert(!state.expire(75500));
  assert(state.expire(75501));
  assert(!state.pending && state.failed && state.failures == 1 && state.retryAt == 85501);
  sameRequest(state.request, request);
  assert(!state.expire(90000) && !state.succeed(request.generation));

  const std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max();
  const std::uint32_t start = maximum - 10;
  const Request wrapping = state.begin(country(0), 0, start);
  assert(!state.expire(static_cast<std::uint32_t>(start + 75000U)));
  assert(state.expire(static_cast<std::uint32_t>(start + 75001U)));
  sameRequest(state.request, wrapping);

  Transaction retry;
  const Request last = retry.begin(country(2), 0, maximum - 5001);
  assert(retry.fail(last.generation, maximum - 5000));
  assert(retry.retryAt == 4999);
  assert(!retry.retryDue(true, maximum - 4000));
  assert(!retry.retryDue(true, 0) && !retry.retryDue(true, 4998));
  assert(retry.retryDue(true, 4999) && retry.retryDue(true, 5000));
  assert(!retry.retryDue(false, 5000));
}

static void testWorkerStartResetsExecutionDeadlineForLatestRequestOnly() {
  Transaction state;
  const Request first = state.begin(country(0), 0, 0);
  const Request next = state.begin(country(2), 0, 1);
  assert(!state.started(first.generation, 70000));
  assert(state.startedAt == 1 && state.pending);
  assert(state.started(next.generation, 70000));
  assert(!state.expire(80000)); // Time spent waiting behind the old fetch is excluded.
  assert(!state.expire(145000));
  assert(state.expire(145001));
  assert(!state.started(next.generation, 145002)); // A late notice cannot resurrect an expired search.
}

static void testGenerationRolloverSkipsZero() {
  Transaction state;
  const std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max();
  state.generation = maximum - 1;
  const Request before = state.begin(country(2), 0, 1);
  assert(before.generation == maximum);
  const Request after = state.begin(country(0), 0, 2);
  assert(after.generation == 1 && state.generation == 1);
  assert(!state.accepts(0) && !state.accepts(before.generation));
  assert(!state.succeed(before.generation) && state.succeed(after.generation));
}

int main() {
  static_assert(std::is_trivially_copyable<Filter>::value, "Filters must remain queue-safe");
  static_assert(std::is_trivially_copyable<Request>::value, "Requests must remain queue-safe");
  testFiltersAndInitialState();
  {
    Transaction cancelled;
    const Request abandoned = cancelled.begin(country(2), 20, 100);
    cancelled.cancel();
    assert(!cancelled.pending && !cancelled.failed && !cancelled.retryDue(true, 1000000));
    assert(!cancelled.accepts(abandoned.generation) && !cancelled.succeed(abandoned.generation));
    assert(!cancelled.fail(abandoned.generation, 101));
    const Request retry = cancelled.begin(country(3), 0, 200);
    assert(cancelled.fail(retry.generation, 201));
    cancelled.cancel();
    assert(!cancelled.failed && !cancelled.failures && !cancelled.retryDue(true, 1000000));
    cancelled.generation = std::numeric_limits<std::uint32_t>::max();
    cancelled.cancel();
    assert(cancelled.generation == 1);
  }
  Transaction refresh;
  auto explicitRefresh = refresh.begin(country(2), 0, 100, true);
  assert(explicitRefresh.refresh);
  auto page = refresh.begin(country(2), 20, 101);
  assert(!page.refresh && !refresh.accepts(explicitRefresh.generation));
  testOutOfOrderResults();
  testReapplyCommittedFilterSupersedesPending();
  testRetryPreservesLatestRequestAndResetsOnSuccess();
  testTimeoutAndClockRollover();
  testGenerationRolloverSkipsZero();
  testWorkerStartResetsExecutionDeadlineForLatestRequestOnly();
  std::puts("Catalog transaction tests passed");
}
