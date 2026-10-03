#pragma once
#include <stdint.h>

namespace freeink {
// Applies only to this refresh. A synchronized grayscale baseline may be reused
// between reading pages; transitions to other content retain physical cleanup.
enum class RefreshContext : uint8_t {
  Normal,
  ContinuousReading,
  TextOnlyAntiAliasing,
  ImageReading,
  // 物理面板方向；调用方不得再次旋转。/ Physical panel directions; consumers must not rotate these again.
  RippleLeft,
  RippleRight,
  RippleUp,
  RippleDown
};
}  // namespace freeink
