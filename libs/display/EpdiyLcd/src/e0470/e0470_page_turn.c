/*
 * SPDX-FileCopyrightText: 2026 mindreset
 * SPDX-License-Identifier: Apache-2.0
 *
 * Physical-coordinate staggered page-turn engine for the E0470 panel.
 * Based on the Read Pico e0470_page_turn component; the rotation-dependent
 * mapping is intentionally owned by the CrossMux caller.
 */
#include "include/e0470_page_turn.h"

#include <stdbool.h>
#include <string.h>

#include "include/e0470_epaper_waveform.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char* TAG = "e0470_turn";
#define TURN_BANDS 16
#define TURN_PHASE_CAP 40
#define TURN_LINE_MAX 2048

static uint8_t (*s_lut)[1024];
static const uint8_t* s_lut_ptr[TURN_PHASE_CAP];
static int s_band0[TURN_BANDS];
static int s_band1[TURN_BANDS];
static int8_t s_band_phase[TURN_BANDS];
static int8_t s_line_phase[TURN_LINE_MAX];
static const EpdWaveformPhases* s_lut_src;
static int s_lut_n;
static int s_tick_us = E0470_TURN_DEFAULT_TICK_US;

// One neutral scan per tick. Pixel actions come from the per-band GL16 LUTs,
// not the ordinary DU table (which would repeat twenty scans per tick).
static const uint8_t s_apply_data[16 * 4] = {0};
static const EpdWaveformPhases s_apply_phases = {
    .phases = 1, .luts = s_apply_data, .phase_times = NULL,
};
static const EpdWaveformPhases* s_apply_ranges[] = {&s_apply_phases};
static const EpdWaveformMode s_apply_mode = {
    .type = MODE_DU, .temp_ranges = 1, .range_data = s_apply_ranges,
};
static const EpdWaveformMode* s_apply_modes[] = {&s_apply_mode};
static const EpdWaveformTempInterval s_apply_intervals[] = {{.min = 0, .max = 50}};
static const EpdWaveform s_apply_waveform = {
    .num_modes = 1, .num_temp_ranges = 1, .mode_data = s_apply_modes,
    .temp_intervals = s_apply_intervals,
};

void e0470_page_turn_release(void) {
    heap_caps_free(s_lut);
    s_lut = NULL;
    s_lut_src = NULL;
    s_lut_n = 0;
}

const char* e0470_turn_dir_name(e0470_turn_dir_t dir) {
    switch (dir) {
        case E0470_TURN_LTR: return "ltr";
        case E0470_TURN_RTL: return "rtl";
        case E0470_TURN_TTB: return "ttb";
        case E0470_TURN_BTT: return "btt";
        default: return "?";
    }
}

void e0470_page_turn_set_tick_us(int us) {
    s_tick_us = us < 0 ? 0 : us;
}

int e0470_page_turn_tick_us(void) {
    return s_tick_us;
}

static void clip_rect(EpdRect* r, int width, int height) {
    if (r->x < 0) {
        r->width += r->x;
        r->x = 0;
    }
    if (r->y < 0) {
        r->height += r->y;
        r->y = 0;
    }
    if (r->x + r->width > width) r->width = width - r->x;
    if (r->y + r->height > height) r->height = height - r->y;
}

static bool assign_bands(int from, int to, int step, bool reverse) {
    const int units = (to - from) / step;
    if (units < TURN_BANDS) return false;
    for (int band = 0; band < TURN_BANDS; ++band) {
        const int position = reverse ? TURN_BANDS - 1 - band : band;
        s_band0[band] = from + units * position / TURN_BANDS * step;
        s_band1[band] = from + units * (position + 1) / TURN_BANDS * step;
    }
    return true;
}

// Exactly mask the physical crop, including individual edge nibbles. Diff
// calculation expands to 32 pixels and vector lookup to 16; neither expansion
// is permission to drive pixels outside the requested reading area.
static void mark_columns(uint8_t* columns, int from, int to) {
    if (from & 1) columns[from++ / 2] |= 0xF0;
    if (to & 1) columns[--to / 2] |= 0x0F;
    if (to > from) memset(columns + from / 2, 0xFF, (size_t)(to - from) / 2);
}

static bool build_luts(const EpdWaveformPhases* gl) {
    if (!s_lut) {
        // The C module owns one lazy 40 KiB PSRAM cache and releases it through
        // e0470_page_turn_release(). Stack/static internal RAM cannot afford it;
        // a failed optional allocation lets the caller use its normal refresh.
        s_lut = (uint8_t (*)[1024])heap_caps_aligned_alloc(16, sizeof(uint8_t[TURN_PHASE_CAP][1024]),
                                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_lut) {
            ESP_LOGW(TAG, "No PSRAM for page-turn phase LUTs");
            return false;
        }
    }
    if (s_lut_src == gl && s_lut_n == gl->phases) return true;
    const int phases = gl->phases < TURN_PHASE_CAP ? gl->phases : TURN_PHASE_CAP;
    for (int phase = 0; phase < phases; ++phase) {
        epd_build_1ppB_lut_1k(s_lut[phase], gl, phase);
        s_lut_ptr[phase] = s_lut[phase];
    }
    s_lut_src = gl;
    s_lut_n = gl->phases;
    return true;
}

static void copy_front_to_back(EpdiyHighlevelState* hl, EpdRect area) {
    const int width = epd_width();
    const int height = epd_height();
    clip_rect(&area, width, height);
    if (area.width <= 0 || area.height <= 0) return;
    for (int y = area.y; y < area.y + area.height; ++y) {
        uint8_t* from = hl->front_fb + y * width / 2;
        uint8_t* to = hl->back_fb + y * width / 2;
        int x = area.x;
        int last = area.x + area.width - 1;
        if (x & 1) {
            to[x / 2] = (uint8_t)((from[x / 2] & 0xF0) | (to[x / 2] & 0x0F));
            ++x;
        }
        if (!(last & 1)) {
            to[last / 2] = (uint8_t)((from[last / 2] & 0x0F) | (to[last / 2] & 0xF0));
            --last;
        }
        if (last >= x) memcpy(to + x / 2, from + x / 2, (size_t)((last - x + 1) / 2));
    }
}

enum EpdDrawError e0470_page_turn(EpdiyHighlevelState* hl, e0470_turn_dir_t dir, int temperature, EpdRect area) {
    epd_clear_phase_luts();
    if (!hl || !hl->front_fb || !hl->back_fb || !hl->difference_fb || !hl->dirty_lines || !hl->dirty_columns)
        return EPD_DRAW_NO_PHASES_AVAILABLE;
    if (dir < E0470_TURN_LTR || dir > E0470_TURN_BTT) dir = E0470_TURN_RTL;

    const EpdWaveformPhases* gl = e0470_waveform_phases(&E0470_WAVEFORM, MODE_GL16);
    if (!gl || !gl->luts || gl->phases <= 0 || gl->phases > TURN_PHASE_CAP) {
        return EPD_DRAW_NO_PHASES_AVAILABLE;
    }

    const int width = epd_width();
    const int height = epd_height();
    if (width <= 0 || (width & 15) || height <= 0 || height > TURN_LINE_MAX)
        return EPD_DRAW_INVALID_CROP;
    if (area.width <= 0 || area.height <= 0) return EPD_DRAW_SUCCESS;
    clip_rect(&area, width, height);
    if (area.width <= 0 || area.height <= 0) return EPD_DRAW_SUCCESS;

    const bool horizontal = dir == E0470_TURN_LTR || dir == E0470_TURN_RTL;
    const bool reverse = dir == E0470_TURN_RTL || dir == E0470_TURN_BTT;
    const bool line_phase = !horizontal;
    if (line_phase) {
        if (!assign_bands(area.y, area.y + area.height, 1, reverse)) return EPD_DRAW_INVALID_CROP;
    } else {
        const int x0 = area.x & ~15;
        const int x1 = (area.x + area.width + 15) & ~15;
        if (!assign_bands(x0, x1, 16, reverse)) return EPD_DRAW_INVALID_CROP;
    }
    if (!build_luts(gl)) return EPD_DRAW_NO_PHASES_AVAILABLE;

    const int ticks = TURN_BANDS + gl->phases - 1;
    const EpdRect full = epd_full_screen();
    const int y0 = area.y;
    const int y1 = area.y + area.height;
    enum EpdDrawError err = EPD_DRAW_SUCCESS;
    const int64_t started = esp_timer_get_time();

    epd_difference_image_cropped(hl->front_fb, hl->back_fb, area, hl->difference_fb,
                                 hl->dirty_lines, hl->dirty_columns);

    for (int tick = 0; tick < ticks; ++tick) {
        const int64_t tick_started = esp_timer_get_time();
        memset(hl->dirty_lines, 0, sizeof(bool) * (size_t)height);
        memset(hl->dirty_columns, 0, (size_t)width / 2);
        memset(s_band_phase, -1, sizeof(s_band_phase));
        if (line_phase) memset(s_line_phase, -1, (size_t)height);
        bool active = false;
        for (int band = 0; band < TURN_BANDS; ++band) {
            const int phase = tick - band;
            if (phase < 0 || phase >= gl->phases) continue;
            const int a = s_band0[band];
            const int b = s_band1[band];
            if (a >= b) continue;
            s_band_phase[band] = (int8_t)phase;
            if (line_phase) {
                memset(hl->dirty_lines + a, 1, (size_t)(b - a));
                memset(s_line_phase + a, (int)(int8_t)phase, (size_t)(b - a));
            } else {
                const int x0 = a < area.x ? area.x : a;
                const int x1 = b > area.x + area.width ? area.x + area.width : b;
                if (x1 > x0) mark_columns(hl->dirty_columns, x0, x1);
            }
            active = true;
        }
        if (!active) continue;

        if (line_phase) {
            mark_columns(hl->dirty_columns, area.x, area.x + area.width);
            epd_set_line_phase_luts(s_lut_ptr, s_line_phase);
        } else {
            memset(hl->dirty_lines + y0, 1, (size_t)(y1 - y0));
            epd_set_col_phase_luts(s_lut_ptr, s_band0, s_band1, s_band_phase, TURN_BANDS);
        }

        err = epd_draw_base(full, hl->difference_fb, full,
                            (enum EpdDrawMode)(MODE_PACKING_1PPB_DIFFERENCE | MODE_DU),
                            temperature, hl->dirty_lines, hl->dirty_columns, &s_apply_waveform);
        epd_clear_phase_luts();
        if (err != EPD_DRAW_SUCCESS) break;
        const int64_t used = esp_timer_get_time() - tick_started;
        if (s_tick_us > 0 && used < s_tick_us) {
            // The scan itself usually dominates. Delay only the remaining
            // budget, keeping the visible ripple cadence stable.
            esp_rom_delay_us((uint32_t)(s_tick_us - used));
        }
    }

    if (err == EPD_DRAW_SUCCESS) copy_front_to_back(hl, area);
    ESP_LOGI(TAG, "dir=%s ticks=%d wall=%d ms err=%d", e0470_turn_dir_name(dir), ticks,
             (int)((esp_timer_get_time() - started) / 1000), (int)err);
    return err;
}
