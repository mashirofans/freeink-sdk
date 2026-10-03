"""Host test for the physical E0470 staggered page-turn engine."""
from pathlib import Path
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]


TEST_C = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "e0470_page_turn.h"

static uint8_t gl_luts[37 * 16 * 4];
static EpdWaveformPhases gl = {37, gl_luts, NULL};
static uint8_t mode_data[16];
static EpdWaveformMode mode = {2, 1, NULL};
static const EpdWaveformMode* modes[1] = {&mode};
static EpdWaveformTempInterval ranges[1] = {{0, 50}};
const EpdWaveform E0470_WAVEFORM = {1, 1, modes, ranges};

static int width = 256, height = 32;
static int calls, staged, fail_after;
static const uint8_t* const* staged_luts;
static EpdiyHighlevelState* active;

const EpdWaveformPhases* e0470_waveform_phases(const EpdWaveform* w, int m) {
    (void)w; (void)m; return &gl;
}
void epd_build_1ppB_lut_1k(uint8_t* lut, const EpdWaveformPhases* p, int f) {
    (void)p; memset(lut, (uint8_t)f, 1024);
}
int epd_width(void) { return width; }
int epd_height(void) { return height; }
EpdRect epd_full_screen(void) { return (EpdRect){0, 0, width, height}; }
void epd_set_col_phase_luts(const uint8_t* const* l, const int* x0, const int* x1,
                            const int8_t* p, int n) {
    (void)x0; (void)x1; (void)p; (void)n; staged_luts = l;
}
void epd_set_line_phase_luts(const uint8_t* const* l, const int8_t* p) {
    (void)p; staged_luts = l;
}
void epd_clear_phase_luts(void) { staged_luts = NULL; }
EpdRect epd_difference_image_cropped(const uint8_t* to, const uint8_t* from,
                                     EpdRect area, uint8_t* diff,
                                     bool* lines, uint8_t* cols) {
    (void)to; (void)from; memset(diff, 0xAA, (size_t)width * height);
    memset(lines, 1, (size_t)height); memset(cols, 0xFF, (size_t)width / 2);
    return area;
}
enum EpdDrawError epd_draw_base(EpdRect area, const uint8_t* data, EpdRect crop,
                                enum EpdDrawMode mode, int temperature,
                                const bool* lines, const uint8_t* cols,
                                const EpdWaveform* waveform) {
    (void)area; (void)data; (void)crop; (void)mode; (void)temperature;
    (void)lines; (void)cols; (void)waveform;
    assert(staged_luts != NULL);
    ++calls;
    if (fail_after && calls >= fail_after) return EPD_DRAW_EMPTY_LINE_QUEUE;
    return EPD_DRAW_SUCCESS;
}

int main(void) {
    uint8_t front[256 * 32 / 2], back[256 * 32 / 2], diff[256 * 32 / 2];
    bool lines[32]; uint8_t cols[128];
    EpdiyHighlevelState hl = {front, back, diff, lines, cols, &E0470_WAVEFORM};
    memset(front, 0x00, sizeof(front));
    memset(back, 0xFF, sizeof(back));
    for (int d = 0; d < 4; ++d) {
        calls = 0; fail_after = 0;
        enum EpdDrawError result = e0470_page_turn(&hl, (e0470_turn_dir_t)d, 20,
                               (EpdRect){0, 0, width, height});
        fprintf(stderr, "dir=%d result=%d calls=%d\n", d, (int)result, calls);
        assert(result == EPD_DRAW_SUCCESS);
        assert(calls == 52); // 16 bands + 37 GL16 phases - 1
    }
    uint8_t old[sizeof(back)]; memcpy(old, back, sizeof(back));
    calls = 0; fail_after = 1;
    assert(e0470_page_turn(&hl, E0470_TURN_RTL, 20,
                           (EpdRect){1, 2, 64, 16}) != EPD_DRAW_SUCCESS);
    assert(memcmp(old, back, sizeof(back)) == 0);
    e0470_page_turn_release();
    return 0;
}
'''


def run():
    with tempfile.TemporaryDirectory(prefix="e0470-page-turn-") as td:
        root = Path(td)
        (root / "freertos").mkdir()
        (root / "esp_attr.h").write_text("#pragma once\n#define IRAM_ATTR\n")
        (root / "esp_log.h").write_text("#pragma once\n#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n")
        (root / "esp_rom_sys.h").write_text("#pragma once\n#include <stdint.h>\nstatic inline void esp_rom_delay_us(uint32_t x){(void)x;}\n")
        (root / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nstatic inline int64_t esp_timer_get_time(void){return 0;}\n")
        (root / "esp_heap_caps.h").write_text(
            "#pragma once\n#include <stdlib.h>\n#define MALLOC_CAP_SPIRAM 1\n#define MALLOC_CAP_8BIT 2\n"
            "static inline void* heap_caps_aligned_alloc(size_t a,size_t n,int c){(void)a;(void)c;return malloc(n);}\n"
            "static inline void heap_caps_free(void*p){free(p);}\n"
        )
        (root / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n")
        (root / "esp_types.h").write_text("#pragma once\n")
        (root / "xtensa").mkdir()
        (root / "xtensa/core-macros.h").write_text("#pragma once\n")
        (root / "freertos/FreeRTOS.h").write_text("#pragma once\ntypedef void* TaskHandle_t; typedef void* SemaphoreHandle_t;\n")
        (root / "freertos/semphr.h").write_text("#pragma once\n#include \"FreeRTOS.h\"\n")
        (root / "freertos/task.h").write_text("#pragma once\n#include \"FreeRTOS.h\"\n")
        test = root / "test_page_turn.c"
        test.write_text(TEST_C)
        binary = root / "page-turn-test"
        includes = [root, SRC / "src/e0470", SRC / "src/e0470/include", SRC / "src/epdiy/src",
                    SRC / "src/epdiy/include"]
        subprocess.run(["cc", "-std=c11", "-D_POSIX_C_SOURCE=200112L", *("-I" + str(p) for p in includes),
                        str(test), str(SRC / "src/e0470/e0470_page_turn.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print("E0470 page-turn directions, phase staging, and failure baseline checks passed")


if __name__ == "__main__":
    run()
