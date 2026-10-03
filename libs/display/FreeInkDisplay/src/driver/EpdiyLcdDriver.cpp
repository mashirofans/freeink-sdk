/*
 * SPDX-FileCopyrightText: 2026 mindreset
 * SPDX-License-Identifier: Apache-2.0
 */

// 中文：epdiy LCD 路径的 PanelDriver 适配。所有面板时序、LUT/波形与逐行供数都在
// EpdiyLcd 库里；这里只把 SDK 的 1bpp 帧缓冲与刷新档位翻译
// 成它的调用。
//
// English: PanelDriver adapter for the epdiy LCD path. All panel timing, LUTs and
// line feeding live in the EpdiyLcd library; this file translates the SDK's
// 1 bpp framebuffer and refresh
// profiles into its calls.

#include "EpdiyLcdDriver.h"

#include <BoardConfig.h>
#include <esp_heap_caps.h>

#include <cstring>

#if FREEINK_DRIVER_EPDIY_LCD

#ifndef FREEINK_EPDIY_LCD_CONFIG
#error \
    "FREEINK_DRIVER_EPDIY_LCD requires a board config: define `const EpdiyLcdConfig& yourConfig();` in namespace freeink and build with -DFREEINK_EPDIY_LCD_CONFIG=yourConfig"
#endif

namespace freeink {

// 板子提供的配置函数，名字由 -DFREEINK_EPDIY_LCD_CONFIG 注入——与 LgfxEpd 的
// FREEINK_LGFX_EPD_CONFIG 同一套路，SDK 因此不需要 include 任何板子头文件。
// / Board-supplied config function, name injected by -DFREEINK_EPDIY_LCD_CONFIG —
// the same pattern as LgfxEpd's FREEINK_LGFX_EPD_CONFIG, so the SDK never includes
// a board header.
const EpdiyLcdConfig& FREEINK_EPDIY_LCD_CONFIG();

namespace {

// Preserve the established expansion convention: a set facade bit maps to
// panel white (15). This wrapper's historical parameter name is blackIsOne.
constexpr bool kBlackIsOne = true;

EpdiyLcdRefresh refreshFor(RefreshMode mode) {
  switch (mode) {
    case RefreshMode::Full:
      return EpdiyLcdRefresh::Full;
    case RefreshMode::Half:
      return EpdiyLcdRefresh::Half;
    case RefreshMode::Fast:
      return EpdiyLcdRefresh::Fast;
  }
  return EpdiyLcdRefresh::Fast;
}

EpdiyLcdRefresh refreshFor(RefreshMode mode, RefreshContext context) {
  if (mode == RefreshMode::Full) return EpdiyLcdRefresh::Full;
  switch (context) {
    case RefreshContext::RippleLeft:
      return EpdiyLcdRefresh::RippleLeft;
    case RefreshContext::RippleRight:
      return EpdiyLcdRefresh::RippleRight;
    case RefreshContext::RippleUp:
      return EpdiyLcdRefresh::RippleUp;
    case RefreshContext::RippleDown:
      return EpdiyLcdRefresh::RippleDown;
    default:
      return refreshFor(mode);
  }
}

bool isRipple(EpdiyLcdRefresh mode) {
  return mode == EpdiyLcdRefresh::RippleLeft || mode == EpdiyLcdRefresh::RippleRight ||
         mode == EpdiyLcdRefresh::RippleUp || mode == EpdiyLcdRefresh::RippleDown;
}

}  // namespace

EpdiyLcdDriver::EpdiyLcdDriver(const EpdiyLcdConfig& cfg) : _cfg(cfg) {}

PanelGeometry EpdiyLcdDriver::geometry() const {
  const uint16_t w = BoardConfig::ACTIVE.displayWidth;
  const uint16_t h = BoardConfig::ACTIVE.displayHeight;
  const uint16_t wb = w / 8;
  return {w, h, wb, static_cast<uint32_t>(wb) * h};
}

GrayscaleCapabilities EpdiyLcdDriver::grayscaleCapabilities(GrayscaleMode mode) const {
  (void)mode;
  // OverlayMasks: plane background 0 = black/white (taken from the B/W base the
  // host supplies first), LSB set = dark, MSB set = light. The host supplies
  // whole LSB/MSB planes; strip uploads are not implemented.
  // Combined: this driver defers the base so the grey commit presents the whole page
  // once. See displayGrayscaleBaseWithContext() below.
  return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Combined, false, false, false};
}

void EpdiyLcdDriver::begin(EpdBus& bus) {
  (void)bus;
  _ready = epdiyLcdBegin(_cfg, BoardConfig::ACTIVE.displayWidth, BoardConfig::ACTIVE.displayHeight, kBlackIsOne);
  if (!_ready) return;

  // 两个选择平面：只有宿主真的做抗锯齿时才会被写入，但分配是无条件的，因为它们
  // 必须在 copyGrayscaleLsb() 第一次被调用之前就位。
  // / The two selector planes: only written when the host actually anti-aliases, but
  // allocated unconditionally because they must exist before the first
  // copyGrayscaleLsb() call.
  const PanelGeometry g = geometry();
  if (_lsb == nullptr) _lsb = static_cast<uint8_t*>(heap_caps_malloc(g.bufferSize, MALLOC_CAP_SPIRAM));
  if (_msb == nullptr) _msb = static_cast<uint8_t*>(heap_caps_malloc(g.bufferSize, MALLOC_CAP_SPIRAM));
  if (_lsb == nullptr || _msb == nullptr) {
    Serial.printf("[EpdiyLcd] grayscale plane allocation failed (%u bytes each)\n",
                  static_cast<unsigned>(g.bufferSize));
    heap_caps_free(_lsb);
    heap_caps_free(_msb);
    _lsb = nullptr;
    _msb = nullptr;
    epdiyLcdEnd();
    _ready = false;
    return;
  }
  memset(_lsb, 0, g.bufferSize);
  memset(_msb, 0, g.bufferSize);
}

void EpdiyLcdDriver::deepSleep(EpdBus& bus) {
  (void)bus;
  _lastBaseMode = EpdiyLcdRefresh::Half;
  epdiyLcdDeepSleep();
  heap_caps_free(_lsb);
  heap_caps_free(_msb);
  _lsb = nullptr;
  _msb = nullptr;
  _ready = false;
}

void EpdiyLcdDriver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  displayWithContext(bus, fb, prev, mode, turnOff, RefreshContext::Normal);
}

void EpdiyLcdDriver::displayWithContext(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode,
                                       bool turnOff, RefreshContext context) {
  (void)bus;
  (void)prev;  // epdiy's highlevel keeps its own previous frame / epdiy 自己记上一帧
  // 独立推屏不能把波纹意图留给后续灰度提交。/ A standalone draw must not leave a ripple intent for a later gray commit.
  _lastBaseMode = refreshFor(mode, context);
  if (!_ready) return;

  // 帧缓冲极性见文件顶部的 kBlackIsOne。/ Framebuffer polarity: see kBlackIsOne above.
  epdiyLcdDraw(fb, refreshFor(mode, context), turnOff);
}

void EpdiyLcdDriver::requestResync(uint8_t settlePasses) {
  (void)settlePasses;
  _lastBaseMode = EpdiyLcdRefresh::Half;
}

void EpdiyLcdDriver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  (void)bus;
  if (_lsb == nullptr || lsb == nullptr) return;
  memcpy(_lsb, lsb, geometry().bufferSize);
}

void EpdiyLcdDriver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  (void)bus;
  if (_msb == nullptr || msb == nullptr) return;
  memcpy(_msb, msb, geometry().bufferSize);
}

void EpdiyLcdDriver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                                 bool factoryMode) {
  (void)bus;
  (void)lut;
  (void)factoryMode;
  const EpdiyLcdRefresh baseMode = _lastBaseMode;
  _lastBaseMode = EpdiyLcdRefresh::Half;
  if (!_ready || _lsb == nullptr || _msb == nullptr) return;

  // `fb` 故意不用：抗锯齿提交时它装的是最后一个选择平面（与页面互补），推上去就是
  // 整屏负片。底图由 EpdiyLcd 在 display() 时留存。
  // / `fb` is deliberately unused: at anti-aliasing commit time it holds the last
  // selector plane (the page's complement), which renders as a screen-wide negative.
  // EpdiyLcd kept the base image during display().
  (void)fb;

  // DU 灰阶太少，实测不可用（文字中间调不到位）。抗锯齿文字页的灰阶推送一律走文字转页档：
  // GL16 加全保持对角线，没变化的像素完全不驱动——原地重推一个黑像素会先把它擦白，那正是
  // 每次翻页白闪的来源。只有 GC16 清屏档原样穿过，因为它本来就要整帧重写，插图页也由阅读器
  // 强制走清屏档保住中间调。
  // / DU carries too few gray levels to be usable (text mid-tones miss). Every
  // anti-aliased text page's gray push goes out as the text turn: GL16 with an all-hold
  // diagonal, so a pixel that did not change is not driven at all -- re-driving a black
  // pixel in place erases it white first, which is what made every turn flash. Only a
  // GC16 clean passes through, because it rewrites the whole frame by design and the
  // reader forces image pages onto it so an illustration keeps its mid-tones.
  //
  // 判据是"不是清屏档"而不是"是 Fast 档"：阅读器在 Read Pico 上把普通翻页提升成了 GL16
  // (Half)，只认 Fast 会让这张表永远挂不上，无闪就失效。
  // / The test is "not the clean profile" rather than "is Fast": the reader promotes an
  // ordinary page turn to GL16 (Half) on Read Pico, so keying on Fast alone would leave
  // the table uninstalled and silently lose the flicker-free turn.
  const EpdiyLcdRefresh grayMode = baseMode == EpdiyLcdRefresh::Full || isRipple(baseMode)
                                     ? baseMode
                                     : EpdiyLcdRefresh::TextTurn;
  epdiyLcdDrawGray(_lsb, _msb, grayMode, turnOff);
}

void EpdiyLcdDriver::displayGrayscaleBaseWithContext(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff,
                                                     RefreshContext context) {
  (void)bus;
  (void)turnOff;
  _lastBaseMode = EpdiyLcdRefresh::Half;
  if (!_ready) return;
  _lastBaseMode = refreshFor(fallback, context);
  // 底图只暂存。宿主接下来会写 LSB/MSB 平面并调 displayGray()，由它合成整页后推一次。
  // / Defer: the host writes the LSB/MSB planes next and calls displayGray(), which
  // composes the whole page and presents it once.
  epdiyLcdStashBase(fb);
}

PanelDriver& epdiyLcdDriver() {
  static EpdiyLcdDriver driver(FREEINK_EPDIY_LCD_CONFIG());
  return driver;
}

}  // namespace freeink

#endif  // FREEINK_DRIVER_EPDIY_LCD
