#pragma once

#include <stdint.h>

namespace RadioAudio {
enum class MetadataType : uint8_t { Title, Bitrate, Info };

// Implemented by the audio owner. These bridges only copy data or update the
// PCM timestamp; callbacks must never call public Audio methods or draw the UI.
void metadata(MetadataType type, const char* text);
void decodedFrame(int32_t samples);
}
