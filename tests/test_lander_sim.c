// Host tests for apps/lander/lander_sim.c: the Luminary-style guidance must
// fly the model to a soft landing, both with a continuous throttle and with
// the game's on/off green button, and the 1202 alarm must fire exactly once.
#include <math.h>
#include <stdio.h>
#include "lander_sim.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("FAIL %s:%d ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static void test_ttf_root(void)
{
    // Linear case (J = A = VDG = 0): 6*V*T = -24*(RD - R) -> T = 4*(R - RD)/V.
    float t = -50.0f;
    CHECK(lander_ttf_solve(0, 0, 30.0f, 0, -900.0f, 0, &t), "linear solve");
    CHECK(fabsf(t - (-120.0f)) < 0.05f, "linear root %f", (double)t);
    // Quadratic case: residual must vanish at the returned root.
    t = -80.0f;
    CHECK(lander_ttf_solve(0, -0.05f, 28.0f, 0.5f, -700.0f, 0, &t), "quad solve");
    float f = (6.0f * -0.05f * t + 6.0f * 28.0f + 18.0f * 0.5f) * t + 24.0f * 700.0f;
    CHECK(fabsf(f) < 1.0f, "quad residual %f at %f", (double)f, (double)t);
    // Past the site there is no negative root.
    t = -10.0f;
    CHECK(!lander_ttf_solve(0, 0, 30.0f, 0, 100.0f, 0, &t), "no negative root past site");
}

// mode 0: continuous autopilot. mode 1: on/off throttle like the green button.
static lander_t fly(int mode, int *alarms, int *saw_p66)
{
    lander_t s; lander_reset(&s);
    const float dt = 0.02f;
    bool was = false;
    *alarms = 0; *saw_p66 = 0;
    for (int i = 0; i < 20000 && s.outcome == LANDER_FLYING; i++) {
        lander_guidance(&s, dt);
        float thr = mode == 0 ? s.cmd_thr : (s.thr < s.cmd_thr ? 1.0f : 0.0f);
        lander_step(&s, dt, thr, s.cmd_pitch);
        if (s.alarm_fired && !was) (*alarms)++;
        was = s.alarm_fired;
        if (s.prog == 66) *saw_p66 = 1;
    }
    return s;
}

static void test_flights(void)
{
    for (int mode = 0; mode < 2; mode++) {
        int alarms, p66;
        lander_t s = fly(mode, &alarms, &p66);
        printf("mode %d: outcome %d t=%.1f td_vh=%.2f td_vx=%.2f pitch=%.1fdeg x=%.1f fuel=%.1f%%\n",
               mode, s.outcome, (double)s.t, (double)s.td_vh, (double)s.td_vx,
               (double)(s.td_pitch * 57.3f), (double)s.td_x, (double)(s.fuel * 100.0f));
        CHECK(s.outcome == LANDER_LANDED, "mode %d did not land softly", mode);
        CHECK(fabsf(s.td_x) < 60.0f, "mode %d landed %.1f m from site", mode, (double)s.td_x);
        CHECK(s.fuel > 0.05f, "mode %d fuel %.3f", mode, (double)s.fuel);
        CHECK(alarms == 1, "mode %d: %d alarms", mode, alarms);
        CHECK(p66 == 1, "mode %d never entered P66", mode);
    }
}

static void test_free_fall_crashes(void)
{
    lander_t s; lander_reset(&s);
    for (int i = 0; i < 20000 && s.outcome == LANDER_FLYING; i++)
        lander_step(&s, 0.02f, 0.0f, 0.0f);
    CHECK(s.outcome == LANDER_CRASHED, "min throttle should crash");
}

int main(void)
{
    test_ttf_root();
    test_flights();
    test_free_fall_crashes();
    if (g_fail) { printf("%d failure(s)\n", g_fail); return 1; }
    printf("lander_sim: all passed\n");
    return 0;
}
