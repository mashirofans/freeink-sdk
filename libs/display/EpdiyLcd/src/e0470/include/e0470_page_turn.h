/*
 * SPDX-FileCopyrightText: 2026 mindreset
 * SPDX-License-Identifier: Apache-2.0
 *
 * Physical-coordinate staggered page-turn engine for the E0470 panel.
 */
#pragma once

#include "../../epdiy/src/epd_highlevel.h"
#include "../../epdiy/src/epdiy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    E0470_TURN_LTR = 0,
    E0470_TURN_RTL = 1,
    E0470_TURN_TTB = 2,
    E0470_TURN_BTT = 3,
} e0470_turn_dir_t;

#define E0470_TURN_DEFAULT_TICK_US 21000

const char* e0470_turn_dir_name(e0470_turn_dir_t dir);

void e0470_page_turn_set_tick_us(int us);
int e0470_page_turn_tick_us(void);

/* Release the lazily allocated phase LUT cache. */
void e0470_page_turn_release(void);

/*
 * Present `area` in physical framebuffer coordinates. The caller owns panel
 * power and must keep the framebuffer state stable until this call returns.
 * The function computes one diff and drives each changed pixel through every
 * GL16 phase, staggered across sixteen bands. On failure, back_fb is untouched.
 */
enum EpdDrawError e0470_page_turn(
    EpdiyHighlevelState* hl,
    e0470_turn_dir_t dir,
    int temperature,
    EpdRect area
);

static inline enum EpdDrawError e0470_page_turn_fullscreen(
    EpdiyHighlevelState* hl,
    e0470_turn_dir_t dir,
    int temperature
) {
    return e0470_page_turn(hl, dir, temperature, epd_full_screen());
}

#ifdef __cplusplus
}
#endif
