#include <cassert>
#include <cstdio>
#include <type_traits>
#include "../FrameCanvas.h"

using RadioDisplay::FrameCanvas;
using CanvasTestMemory::state;
static_assert(!std::is_copy_constructible<FrameCanvas>::value, "A framebuffer must not duplicate ownership");
static_assert(!std::is_copy_assignable<FrameCanvas>::value, "A framebuffer must not duplicate ownership");

void checkAllocation() {
  CanvasTestMemory::reset();
  const size_t bytes = size_t(RadioDisplay::ScreenWidth) * RadioDisplay::ScreenHeight * 2;
  {
    FrameCanvas canvas(RadioDisplay::ScreenWidth, RadioDisplay::ScreenHeight);
    assert(canvas.width() == RadioDisplay::ScreenWidth && canvas.height() == RadioDisplay::ScreenHeight);
    if (RadioDisplay::LargeDisplay) {
      assert(bytes == 153600 && !canvas.getBuffer());
      assert(state().baseRequests == 0 && state().capsRequests == 0);
    } else {
      assert(bytes == 40960 && canvas.getBuffer());
      assert(state().baseRequests == 1 && state().baseBytes == bytes && state().capsRequests == 0);
    }
    assert(canvas.allocate());
    uint16_t* const buffer = canvas.getBuffer();
    assert(buffer);
    if (RadioDisplay::LargeDisplay) {
      assert(state().baseRequests == 0 && state().capsRequests == 1);
      assert(state().capsCount == 1 && state().capsSize == bytes);
      assert(state().caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    for (size_t pixel = 0; pixel < bytes / 2; ++pixel) assert(buffer[pixel] == 0);
    buffer[0] = 0x1234; buffer[bytes / 2 - 1] = 0xabcd;
    assert(canvas.allocate() && canvas.getBuffer() == buffer);
    assert(buffer[0] == 0x1234 && buffer[bytes / 2 - 1] == 0xabcd);
    assert(state().capsRequests == (RadioDisplay::LargeDisplay ? 1U : 0U));
    assert(state().baseRequests == (RadioDisplay::LargeDisplay ? 0U : 1U));
  }
  assert(!state().capsLive && !state().baseLive);
  assert(state().baseDestructors == 1);
  assert(state().baseFrees == (RadioDisplay::LargeDisplay ? 0U : 1U));
  assert(state().capsFrees == (RadioDisplay::LargeDisplay ? 1U : 0U));
  assert(state().borrowedDestructors == (RadioDisplay::LargeDisplay ? 1U : 0U));
  assert(state().unreleasedBorrowed == 0); // Derived destructor clears its released pointer.
}
void checkFailure() {
  CanvasTestMemory::reset();
  state().failBase = true; state().failCaps = true;
  {
    FrameCanvas canvas(RadioDisplay::ScreenWidth, RadioDisplay::ScreenHeight);
    assert(!canvas.getBuffer() && !canvas.allocate());
    assert(!canvas.getBuffer() && !state().baseLive && !state().capsLive);
    if (RadioDisplay::LargeDisplay) {
      assert(state().capsRequests == 1 && state().baseRequests == 0);
      assert(state().caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      // Failure may be retried, but never spills into the internal allocator.
      state().failCaps = false;
      assert(canvas.allocate() && state().capsRequests == 2 && state().baseRequests == 0);
    } else {
      // Compact mode preserves the base library allocation/failure behavior.
      assert(state().baseRequests == 1 && state().capsRequests == 0);
      state().failBase = false;
      assert(!canvas.allocate() && state().baseRequests == 1 && state().capsRequests == 0);
    }
  }
  assert(!state().capsLive && !state().baseLive && state().unreleasedBorrowed == 0);
  assert(state().baseFrees == 0);
  assert(state().capsFrees == (RadioDisplay::LargeDisplay ? 1U : 0U));
}
void checkUnused() {
  CanvasTestMemory::reset();
  { FrameCanvas unused(RadioDisplay::ScreenWidth, RadioDisplay::ScreenHeight); }
  assert(state().capsRequests == 0 && state().capsFrees == 0);
  assert(state().baseRequests == (RadioDisplay::LargeDisplay ? 0U : 1U));
  assert(state().baseFrees == (RadioDisplay::LargeDisplay ? 0U : 1U));
  assert(state().baseDestructors == 1 && state().unreleasedBorrowed == 0);
}
int main() {
  checkAllocation(); checkFailure(); checkUnused();
  std::puts("FrameCanvas allocation and ownership tests passed");
}
