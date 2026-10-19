#pragma once

#include "RadioModel.h"

namespace RadioStreaming {
// Resolve only the connection URL. Station identity and its official URL remain
// unchanged, so retries and future sessions can follow a provider's new CDN.
bool resolvePlaybackUrl(const RadioModel::Station& station, char (&out)[512], char (&error)[80]);
} // namespace RadioStreaming
