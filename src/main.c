// fw2-lander — lunar landing game on a DSKY-style display.
//
// The descent is flown by you; the Apollo 11 guidance (Luminary 099, P64
// quadratic guidance then P66 rate-of-descent; see lander_sim.h) runs
// alongside and shows what it would command on the GUID panel.
//
//   Tilt the board left/right ... steer (LM pitch), BMI323 accelerometer.
//   Hold GREEN .................. throttle up (release = DPS minimum 10%).
//   RED ......................... RSET: clear PROG/RESTART after the 1202.
//   NAV LEFT/RIGHT .............. trim pitch (also the fallback with no IMU);
//                                 NAV CENTER zeroes the trim.
//   Hold HOME 5 s ............... leave the app (FW2App contract).
//
// Before the first flight the app learns how you hold the board: hold it
// level and press GREEN, then tilt it right and press GREEN.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "fw2.h"
#include "platform/diag.h"
#include "platform/psram.h"
#include "lander_sim.h"

#define BE(c) ((uint16_t)((((c) >> 8) & 0xFF) | (((c) & 0xFF) << 8)))
#define RAD2DEG 57.29578f
#define M2FT    3.28084f
#define KEY_FRESH_MS 1200   // > the keyboard's ~500 ms held-button report cadence

// Colours (RGB565 -> wire order)
#define C_BLACK   BE(0x0000)
#define C_PANEL   BE(0x2945)
#define C_EL      BE(0x57EF)   // DSKY electroluminescent green
#define C_GHOST   BE(0x00E2)   // unlit segment
#define C_LAMPOFF BE(0x3186)
#define C_LAMPTXT BE(0x6B4D)
#define C_WHITE   BE(0xFFFF)
#define C_AMBER   BE(0xFD20)
#define C_GREY    BE(0x8410)
#define C_MAGENTA BE(0xF81F)
#define C_RED     BE(0xF800)
// Front-panel button colours, already in wire order (AGENTS.md)
#define BTN_GREEN_BG 0x0012
#define BTN_RED_BG   0x0780

// ---------------------------------------------------------------- 7-segment
#define SEG_W 22
#define SEG_H 34
#define SEG_T 4
#define CELL  30

static const uint8_t k_digit_segs[10] = {   // bit0=a .. bit6=g
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

static void seg_char(int x, int y, char ch)
{
    uint8_t m = 0;
    if (ch >= '0' && ch <= '9') m = k_digit_segs[ch - '0'];
    else if (ch == '-' || ch == '+') m = 0x40;
    const int h2 = SEG_H / 2, v = h2 - SEG_T - SEG_T / 2;
    st7796_fill_rect(x, y, SEG_W, SEG_H, C_BLACK);
    struct { int x, y, w, h; } r[7] = {
        { x + SEG_T, y, SEG_W - 2 * SEG_T, SEG_T },                      // a
        { x + SEG_W - SEG_T, y + SEG_T, SEG_T, v },                      // b
        { x + SEG_W - SEG_T, y + h2 + SEG_T / 2, SEG_T, v },             // c
        { x + SEG_T, y + SEG_H - SEG_T, SEG_W - 2 * SEG_T, SEG_T },      // d
        { x, y + h2 + SEG_T / 2, SEG_T, v },                             // e
        { x, y + SEG_T, SEG_T, v },                                      // f
        { x + SEG_T, y + h2 - SEG_T / 2, SEG_W - 2 * SEG_T, SEG_T },     // g
    };
    for (int i = 0; i < 7; i++)
        st7796_fill_rect(r[i].x, r[i].y, r[i].w, r[i].h, (m >> i) & 1 ? C_EL : C_GHOST);
    if (ch == '+')
        st7796_fill_rect(x + SEG_W / 2 - SEG_T / 2, y + SEG_T * 2, SEG_T, SEG_H - SEG_T * 4, C_EL);
}

// Field cache: only redraw characters that changed.
enum { F_PROG, F_VERB, F_NOUN, F_R1, F_R2, F_R3, F_COUNT };
static const struct { int x, y, n; } k_field[F_COUNT] = {
    { 296, 16, 2 }, { 156, 66, 2 }, { 296, 66, 2 },
    { 190, 114, 6 }, { 190, 166, 6 }, { 190, 218, 6 },
};
static char s_field[F_COUNT][7];

static void field_set(int f, const char *str)
{
    int len = (int)strlen(str);
    for (int i = 0; i < k_field[f].n; i++) {
        char c = i < len ? str[i] : ' ';
        if (s_field[f][i] != c) {
            s_field[f][i] = c;
            seg_char(k_field[f].x + i * CELL, k_field[f].y, c);
        }
    }
}

static void field_invalidate(void) { memset(s_field, 0, sizeof s_field); }

// Signed 5-digit DSKY register text: "+01234" / "-00042".
static void reg_text(char out[7], int v, bool sign)
{
    if (v > 99999) v = 99999;
    if (v < -99999) v = -99999;
    snprintf(out, 7, "%c%05d", sign ? (v < 0 ? '-' : '+') : ' ', v < 0 ? -v : v);
}

// ---------------------------------------------------------------- lamps
typedef struct { const char *l1, *l2; bool amber; } lamp_t;
enum { L_UPLINK, L_NOATT, L_STBY, L_KEYREL, L_OPRERR, L_TEMP, L_GIMBAL,
       L_PROG, L_RESTART, L_TRACKER, L_ALT, L_VEL, L_COUNT };
static const lamp_t k_lamp[L_COUNT] = {
    { "UPLINK", "ACTY", false }, { "NO ATT", "", false }, { "STBY", "", false },
    { "KEY REL", "", false },    { "OPR ERR", "", false },
    { "TEMP", "", true },        { "GIMBAL", "LOCK", true }, { "PROG", "", true },
    { "RESTART", "", true },     { "TRACKER", "", true },    { "ALT", "", true },
    { "VEL", "", true },
};
static int8_t s_lamp[L_COUNT];   // -1 = never drawn

static void lamp_xy(int i, int *x, int *y)
{
    int col = i < L_TEMP ? 0 : 1, row = col ? i - L_TEMP : i;
    *x = 4 + col * 71;
    *y = 4 + row * 41;
}

static void text_centered(int x, int y, int w, const char *s, uint16_t fg, uint16_t bg)
{
    int tw = (int)strlen(s) * 6;
    st7796_draw_text(x + (w - tw) / 2, y, 1, fg, bg, s);
}

static void lamp_set(int i, bool on)
{
    if (s_lamp[i] == (int8_t)on) return;
    s_lamp[i] = (int8_t)on;
    int x, y; lamp_xy(i, &x, &y);
    uint16_t bg = on ? (k_lamp[i].amber ? C_AMBER : C_WHITE) : C_LAMPOFF;
    uint16_t fg = on ? C_BLACK : C_LAMPTXT;
    st7796_fill_rect(x, y, 67, 37, bg);
    if (k_lamp[i].l2[0]) {
        text_centered(x, y + 9, 67, k_lamp[i].l1, fg, bg);
        text_centered(x, y + 20, 67, k_lamp[i].l2, fg, bg);
    } else {
        text_centered(x, y + 15, 67, k_lamp[i].l1, fg, bg);
    }
}

// ---------------------------------------------------------------- static art
static void label_box(int x, int y, const char *s)   // green lit legend
{
    int w = (int)strlen(s) * 6 + 6;
    st7796_fill_rect(x, y, w, 10, C_EL);
    st7796_draw_text(x + 3, y + 1, 1, C_BLACK, C_EL, s);
}

static bool s_compacty = true;
static void comp_acty(bool on)
{
    if (s_compacty == on) return;
    s_compacty = on;
    uint16_t bg = on ? C_EL : C_BLACK, fg = on ? C_BLACK : C_GHOST;
    st7796_fill_rect(156, 10, 62, 44, bg);
    text_centered(156, 20, 62, "COMP", fg, bg);
    text_centered(156, 34, 62, "ACTY", fg, bg);
}

static void draw_static(void)
{
    st7796_fill_screen(C_BLACK);
    st7796_fill_rect(146, 0, 236, 258, C_PANEL);          // DSKY body
    st7796_fill_rect(150, 4, 228, 250, C_BLACK);          // EL window
    label_box(298, 5, "PROG");
    label_box(158, 55, "VERB");
    label_box(298, 55, "NOUN");
    for (int i = 0; i < 3; i++) {
        int y = 108 + i * 52;
        st7796_fill_rect(156, y, 216, 2, C_EL);
    }
    // GUID panel frame
    st7796_draw_text(388, 4, 1, C_GREY, C_BLACK, "GUID");
    st7796_fill_rect(390, 16, 28, 184, C_GREY);
    st7796_draw_text(392, 204, 1, C_GREY, C_BLACK, "THR");
    st7796_draw_text(388, 248, 1, C_GREY, C_BLACK, "ATT");
    memset(s_lamp, -1, sizeof s_lamp);
    for (int i = 0; i < L_COUNT; i++) lamp_set(i, false);
    s_compacty = true; comp_acty(false);
    field_invalidate();
}

// Register legends: what R1..R3 currently mean (5 chars wide, 2 lines).
static void reg_labels(const char *const l[6])
{
    for (int i = 0; i < 3; i++) {
        st7796_fill_rect(152, 118 + i * 52, 36, 24, C_BLACK);
        st7796_draw_text(154, 120 + i * 52, 1, C_GREY, C_BLACK, l[2 * i]);
        st7796_draw_text(154, 132 + i * 52, 1, C_GREY, C_BLACK, l[2 * i + 1]);
    }
}
static const char *const k_lbl_fly[6] = { "ALT", "FT", "HDOT", ".1FPS", "FUEL", "PCT" };
static const char *const k_lbl_cal[6] = { "ACC X", "MG", "ACC Y", "MG", "ACC Z", "MG" };
static const char *const k_lbl_end[6] = { "RANGE", "FT", "HDOT", ".1FPS", "FUEL", "PCT" };

// ---------------------------------------------------------------- message + buttons
static char s_msg[40];
static void message(const char *m)
{
    if (strcmp(m, s_msg) == 0) return;
    snprintf(s_msg, sizeof s_msg, "%s", m);
    st7796_fill_rect(146, 262, 334, 30, C_BLACK);
    st7796_draw_text(150, 268, 2, C_EL, C_BLACK, s_msg);
}

static void button_bar(const char *green, const char *red)
{
    st7796_fill_rect(0, 296, ST7796_W, 24, C_BLACK);
    const int w = (ST7796_W - 12) / 5;
    if (green) {
        st7796_fill_rect(2 * 96, 296, w, 24, BTN_GREEN_BG);
        int tw = (int)strlen(green) * 12;
        st7796_draw_text(2 * 96 + (w - tw) / 2, 300, 2, C_WHITE, BTN_GREEN_BG, green);
    }
    if (red) {
        st7796_fill_rect(4 * 96, 296, w, 24, BTN_RED_BG);
        int tw = (int)strlen(red) * 12;
        st7796_draw_text(4 * 96 + (w - tw) / 2, 300, 2, C_WHITE, BTN_RED_BG, red);
    }
}

// ---------------------------------------------------------------- GUID panel
static void guid_panel(const lander_t *s)
{
    // Throttle bar: actual fill, guidance tick.
    const int bx = 392, by = 18, bw = 24, bh = 180;
    int fh = (int)(s->thr * bh + 0.5f);
    st7796_fill_rect(bx, by, bw, bh - fh, C_BLACK);
    st7796_fill_rect(bx, by + bh - fh, bw, fh, C_AMBER);
    if (s->guid_valid) {
        int ty = by + bh - (int)(s->cmd_thr * bh + 0.5f);
        if (ty > by + bh - 3) ty = by + bh - 3;
        st7796_fill_rect(bx - 2, ty, bw + 4, 3, C_MAGENTA);
    }
    // Attitude scale: +/-45 deg over +/-40 px. White = LM, magenta = guidance.
    const int cx = 430, sy = 222, half = 40;
    st7796_fill_rect(cx - half - 4, sy, 2 * half + 8, 22, C_BLACK);
    st7796_fill_rect(cx - half, sy + 10, 2 * half, 2, C_GREY);
    st7796_fill_rect(cx - 1, sy + 4, 2, 14, C_GREY);
    int ax = cx + (int)(s->pitch / LANDER_PITCH_MAX * half);
    st7796_fill_rect(ax - 2, sy, 4, 22, C_WHITE);
    if (s->guid_valid) {
        int gx = cx + (int)(s->cmd_pitch / LANDER_PITCH_MAX * half);
        st7796_fill_rect(gx - 4, sy + 7, 8, 8, C_MAGENTA);
    }
    // Numbers
    char b[12];
    snprintf(b, sizeof b, "%-6d", (int)(-s->ttf));
    st7796_draw_text(424, 18, 1, C_GREY, C_BLACK, "TTF S");
    st7796_draw_text(424, 28, 1, C_WHITE, C_BLACK, s->prog == 64 ? b : "--    ");
    snprintf(b, sizeof b, "%-6d", (int)(-s->x * M2FT));
    st7796_draw_text(424, 48, 1, C_GREY, C_BLACK, "RNG FT");
    st7796_draw_text(424, 58, 1, C_WHITE, C_BLACK, b);
    snprintf(b, sizeof b, "%-+6d", (int)(s->vx * M2FT));
    st7796_draw_text(424, 78, 1, C_GREY, C_BLACK, "HV FPS");
    st7796_draw_text(424, 88, 1, C_WHITE, C_BLACK, b);
    snprintf(b, sizeof b, "%-+6d", (int)(s->pitch * RAD2DEG));
    st7796_draw_text(424, 108, 1, C_GREY, C_BLACK, "PIT DG");
    st7796_draw_text(424, 118, 1, C_WHITE, C_BLACK, b);
}

// ---------------------------------------------------------------- LEDs
static void leds_show(int n_lit, rgb_t lit, rgb_t rest)
{
    for (int i = 0; i < FW2_LED_COUNT; i++)
        ws2812_set_pixel(i, i < n_lit ? lit : rest);
    ws2812_show();
}

// ---------------------------------------------------------------- IMU steering
static bool s_have_imu;
static float s_g[3];              // latest accel, g
static float s_n[3], s_u[3];      // calibrated "level" and "right" axes

static void imu_poll(void)
{
    bmi323_reading_t r;
    if (s_have_imu && bmi323_read(&r) && r.valid) {
        // light low-pass to take the edge off hand jitter
        s_g[0] += 0.3f * (r.ax - s_g[0]);
        s_g[1] += 0.3f * (r.ay - s_g[1]);
        s_g[2] += 0.3f * (r.az - s_g[2]);
    }
}

static float dot3(const float *a, const float *b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static void norm3(float *a)
{
    float m = sqrtf(dot3(a, a));
    if (m > 1e-6f) { a[0] /= m; a[1] /= m; a[2] /= m; }
}

static void cal_level(void) { memcpy(s_n, s_g, sizeof s_n); norm3(s_n); }

static void cal_right(void)
{
    float u[3]; memcpy(u, s_g, sizeof u);
    float k = dot3(u, s_n);
    for (int i = 0; i < 3; i++) u[i] -= k * s_n[i];
    if (sqrtf(dot3(u, u)) < 0.15f) {
        // Not tilted enough to learn a direction: fall back to the board
        // axis most perpendicular to "level" (direction unverified).
        int best = 0;
        for (int i = 1; i < 3; i++) if (fabsf(s_n[i]) < fabsf(s_n[best])) best = i;
        float e[3] = { 0, 0, 0 }; e[best] = 1.0f;
        k = dot3(e, s_n);
        for (int i = 0; i < 3; i++) u[i] = e[i] - k * s_n[i];
        DIAG("lander: right-tilt too small, fallback axis %d\n", best);
    }
    norm3(u);
    memcpy(s_u, u, sizeof s_u);
    DIAG("lander: cal n=%d,%d,%d u=%d,%d,%d (milli)\n",
         (int)(s_n[0] * 1000), (int)(s_n[1] * 1000), (int)(s_n[2] * 1000),
         (int)(s_u[0] * 1000), (int)(s_u[1] * 1000), (int)(s_u[2] * 1000));
}

static float tilt_rad(void)
{
    if (!s_have_imu) return 0.0f;
    float t = atan2f(dot3(s_g, s_u), dot3(s_g, s_n));
    const float dead = 2.0f / RAD2DEG;
    if (fabsf(t) < dead) return 0.0f;
    return t > 0 ? t - dead : t + dead;
}

// ---------------------------------------------------------------- main
typedef enum { ST_CAL_LEVEL, ST_CAL_RIGHT, ST_FLY, ST_END } state_t;

static uint32_t rng = 0x1202u;
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 16; }

int main(void)
{
    board_init();
    fw2_app_recovery_init();
    size_t psram_bytes = psram_init();
    st7796_init();
    fw2_app_about_use_lcd();

    fw2kb_t kb;
    fw2kb_init(&kb);
    bool agentio = psram_bytes >= (size_t)ST7796_W * ST7796_H * 2;
    if (agentio) {               // shadow framebuffer lives in PSRAM
        agentio_init();
        agentio_bind_keyboard(&kb);
    }
    st7796_fill_screen(C_BLACK);   // whole surface before the backlight (FW2App rule 4)
    draw_static();
    board_backlight_set(1);
    picpwr_release_unused();

    ws2812_init(pio1, 0, PIN_LED_DATA);
    ws2812_set_brightness(32);
    ws2812_clear();
    ws2812_show();

    // Rails take ~1 s to apply after recovery_init requested SENSORS.
    message("LUMINARY 099");
    uint64_t t0 = time_us_64();
    while (!(s_have_imu = bmi323_init()) && time_us_64() - t0 < 2000000) {
        fw2_app_recovery_task();
        picpwr_task();
        if (agentio) agentio_task();
        fw2_app_recovery_sleep_ms(50);
    }
    DIAG("lander: v002 up, imu=%d agentio=%d\n", s_have_imu, agentio);

    lander_t s;
    lander_reset(&s);
    state_t st = s_have_imu ? ST_CAL_LEVEL : ST_FLY;
    bool st_entered = false;
    bool prog_latch = false, restart_latch = false;
    float nav_trim = 0.0f;
    int callout = 0;           // 0 none, 1 = 60 s shown, 2 = 30 s shown
    float callout_t = 0.0f;
    float acc = 0.0f;
    uint64_t last = time_us_64(), t_reg = 0, t_fast = 0, t_led = 0, t_diag = 0;

    for (;;) {
        fw2_app_recovery_task();
        picpwr_task();

        uint64_t now = time_us_64();
        float dt = (float)(now - last) * 1e-6f;
        last = now;
        if (dt > 0.1f) dt = 0.1f;   // e.g. after a screenshot stalled us

        imu_poll();

        bool green_edge = false, red_edge = false;
        uartkbd_event_t ev;
        while (uartkbd_next_event(&ev)) {
            if (!ev.pressed) continue;
            if (ev.btn == UARTKBD_BTN_GREEN) green_edge = true;
            if (ev.btn == UARTKBD_BTN_RED) red_edge = true;
            if (ev.btn == UARTKBD_BTN_NAV_CENTER) nav_trim = 0.0f;
        }
        bool green_held = uartkbd_button_down_fresh(UARTKBD_BTN_GREEN, KEY_FRESH_MS);
        if (uartkbd_button_down_fresh(UARTKBD_BTN_NAV_LEFT, KEY_FRESH_MS))  nav_trim -= 0.5f * dt;
        if (uartkbd_button_down_fresh(UARTKBD_BTN_NAV_RIGHT, KEY_FRESH_MS)) nav_trim += 0.5f * dt;
        if (nav_trim > LANDER_PITCH_MAX) nav_trim = LANDER_PITCH_MAX;
        if (nav_trim < -LANDER_PITCH_MAX) nav_trim = -LANDER_PITCH_MAX;

        switch (st) {
        case ST_CAL_LEVEL:
        case ST_CAL_RIGHT:
            if (!st_entered) {
                st_entered = true;
                button_bar("SET", NULL);
                reg_labels(k_lbl_cal);
                message(st == ST_CAL_LEVEL ? "HOLD LEVEL, PRESS GREEN"
                                           : "TILT RIGHT, PRESS GREEN");
                field_set(F_PROG, "00");
                field_set(F_VERB, "16");
                field_set(F_NOUN, "04");
                lamp_set(L_STBY, true);
            }
            if (now - t_reg > 250000) {     // live accel in milli-g
                t_reg = now;
                char b[7];
                reg_text(b, (int)(s_g[0] * 1000), true); field_set(F_R1, b);
                reg_text(b, (int)(s_g[1] * 1000), true); field_set(F_R2, b);
                reg_text(b, (int)(s_g[2] * 1000), true); field_set(F_R3, b);
                leds_show(0, (rgb_t){0}, (rgb_t){ .b = 40 });
            }
            if (green_edge) {
                if (st == ST_CAL_LEVEL) { cal_level(); st = ST_CAL_RIGHT; }
                else { cal_right(); st = ST_FLY; }
                st_entered = false;
            }
            break;

        case ST_FLY: {
            if (!st_entered) {
                st_entered = true;
                lander_reset(&s);
                prog_latch = restart_latch = false;
                callout = 0; acc = 0.0f;
                lamp_set(L_STBY, false);
                button_bar("THRUST", "RSET");
                reg_labels(k_lbl_fly);
                message(s_have_imu ? "P64 TILT+GREEN" : "P64 NAV L/R+GREEN");
                DIAG("lander: P64 start\n");
            }
            if (red_edge) {                       // RSET
                prog_latch = restart_latch = false;
                if (lander_alarm_active(&s)) s.alarm_age = LANDER_ALARM_S;
            }
            float pitch_target = tilt_rad() * 1.3f + nav_trim;
            acc += dt;
            while (acc >= 0.02f) {                // fixed 50 Hz model
                acc -= 0.02f;
                bool fired = s.alarm_fired;
                lander_guidance(&s, 0.02f);
                lander_step(&s, 0.02f, green_held ? 1.0f : 0.0f, pitch_target);
                if (!fired && s.alarm_fired) {
                    prog_latch = restart_latch = true;
                    DIAG("lander: PROGRAM ALARM 1202 at %d ft\n", (int)(s.h * M2FT));
                }
            }
            bool alarm = lander_alarm_active(&s);
            bool flash = (now / 400000) & 1;

            if (now - t_reg > 250000) {           // DSKY registers, 4 Hz
                t_reg = now;
                char b[7];
                snprintf(b, sizeof b, "%02d", s.prog);
                field_set(F_PROG, b);
                if (alarm) {
                    field_set(F_VERB, flash ? "05" : "  ");
                    field_set(F_NOUN, flash ? "09" : "  ");
                    bool bail = s.alarm_age < LANDER_BAILOUT_S;
                    field_set(F_R1, bail ? "      " : " 01202");
                    field_set(F_R2, bail ? "      " : " 00000");
                    field_set(F_R3, bail ? "      " : " 01202");
                } else {
                    field_set(F_VERB, "06");
                    field_set(F_NOUN, "60");
                    reg_text(b, (int)(s.h * M2FT + 0.5f), true);          field_set(F_R1, b);
                    reg_text(b, (int)lroundf(s.vh * M2FT * 10.0f), true); field_set(F_R2, b);
                    reg_text(b, (int)(s.fuel * 100.0f + 0.5f), true);     field_set(F_R3, b);
                }
            }
            if (now - t_fast > 50000) {           // lamps + GUID, 20 Hz
                t_fast = now;
                comp_acty(s.guid_valid && (rnd() & 3) == 0);
                lamp_set(L_PROG, prog_latch);
                lamp_set(L_RESTART, restart_latch &&
                         (s.alarm_age < LANDER_BAILOUT_S || flash));
                lamp_set(L_GIMBAL, fabsf(s.pitch) > 0.70f);
                lamp_set(L_ALT, s.vh < -(0.1f * s.h + 3.0f));
                lamp_set(L_VEL, s.h < 60.0f && fabsf(s.vx) > 1.2f);
                guid_panel(&s);

                // Messages, most important first.
                float hover_s = s.fuel * LANDER_BURN_S / (LANDER_G_MOON / LANDER_AMAX);
                if (callout < 1 && hover_s < 60.0f) { callout = 1; callout_t = 0; }
                if (callout < 2 && hover_s < 30.0f) { callout = 2; callout_t = 0; }
                callout_t += 0.05f;
                if (alarm)                          message("1202 PROGRAM ALARM");
                else if (s.fuel <= 0.0f)            message("FUEL DEPLETED");
                else if (callout && callout_t < 3)  message(callout == 1 ? "60 SECONDS" : "30 SECONDS");
                else if (s.h < 1.7f)                message("CONTACT LIGHT");
                else if (s.prog == 66)              message("P66 RATE OF DESCENT");
                else                                message("P64 APPROACH");
            }
            if (now - t_led > 50000) {
                t_led = now;
                if (alarm) leds_show(flash ? FW2_LED_COUNT : 0, (rgb_t){ .r = 255 }, (rgb_t){0});
                else leds_show((int)(s.thr * FW2_LED_COUNT + 0.5f),
                               (rgb_t){ .r = 255, .g = 110 }, (rgb_t){0});
            }
            if (now - t_diag > 1000000) {
                t_diag = now;
                DIAG("P%d h=%dft vh=%d(0.1fps) x=%dft vx=%dfps thr=%d%% pit=%d cmd=%d%%/%d fuel=%d%% ttf=%d\n",
                     s.prog, (int)(s.h * M2FT), (int)(s.vh * M2FT * 10), (int)(s.x * M2FT),
                     (int)(s.vx * M2FT), (int)(s.thr * 100), (int)(s.pitch * RAD2DEG),
                     (int)(s.cmd_thr * 100), (int)(s.cmd_pitch * RAD2DEG),
                     (int)(s.fuel * 100), (int)s.ttf);
            }
            if (s.outcome != LANDER_FLYING) { st = ST_END; st_entered = false; }
            break;
        }

        case ST_END:
            if (!st_entered) {
                st_entered = true;
                bool ok = s.outcome == LANDER_LANDED;
                char b[7];
                field_set(F_VERB, "06");
                field_set(F_NOUN, "60");
                reg_text(b, (int)(s.td_x * M2FT), true);                 field_set(F_R1, b);
                reg_text(b, (int)lroundf(s.td_vh * M2FT * 10.0f), true); field_set(F_R2, b);
                reg_text(b, (int)(s.fuel * 100.0f + 0.5f), true);        field_set(F_R3, b);
                comp_acty(false);
                lamp_set(L_ALT, false); lamp_set(L_VEL, false);
                guid_panel(&s);
                char m[40];
                if (ok) snprintf(m, sizeof m, "THE EAGLE HAS LANDED");
                else    snprintf(m, sizeof m, "LM LOST AT %d.%d FPS",
                                 (int)(-s.td_vh * M2FT), (int)(-s.td_vh * M2FT * 10) % 10);
                message(m);
                button_bar("AGAIN", NULL);
                reg_labels(k_lbl_end);
                leds_show(FW2_LED_COUNT, ok ? (rgb_t){ .g = 255 } : (rgb_t){ .r = 255 }, (rgb_t){0});
                DIAG("lander: %s td_vh=%d(0.1fps) td_vx=%dfps pitch=%d x=%dft fuel=%d%% t=%ds\n",
                     ok ? "LANDED" : "CRASHED", (int)(s.td_vh * M2FT * 10), (int)(s.td_vx * M2FT),
                     (int)(s.td_pitch * RAD2DEG), (int)(s.td_x * M2FT), (int)(s.fuel * 100), (int)s.t);
            }
            if (green_edge) {
                draw_static();
                st = ST_FLY;
                st_entered = false;
            }
            break;
        }

        if (agentio) agentio_task();
    }
}
