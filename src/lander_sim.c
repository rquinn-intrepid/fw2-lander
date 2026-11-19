#include "lander_sim.h"
#include <math.h>

// APPRQUAD (P64) targets, relative to the landing site. Values are chosen
// for this game; the real ones were mission-loaded erasables.
#define RDG_X    0.0f
#define RDG_H   30.0f
#define VDG_X    0.5f
#define VDG_H   -1.0f
#define ADG_X   -0.05f
#define ADG_H    0.05f
#define JDG_X    0.0f
#define TAUVERT  1.5f    // P66 vertical-rate time constant, s
#define TAUHORIZ 3.0f    // P66 horizontal nulling time constant, s
#define THR_TAU  0.25f   // throttle response, s
#define PITCH_RATE 0.44f // attitude rate limit, rad/s (25 deg/s)

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float lander_self_right(float pitch)
{
    float m = fabsf(pitch) - LANDER_RIGHTING_DEADZONE;
    if (m <= 0.0f) return 0.0f;
    return pitch > 0.0f ? m : -m;
}

void lander_reset(lander_t *s)
{
    *s = (lander_t){0};
    s->x = -700.0f;  s->h = 1000.0f;   // ~3300 ft, 2300 ft uprange of the site
    s->vx = 28.0f;   s->vh = -20.0f;
    s->thr = 0.5f;   s->pitch = -0.4f;
    s->fuel = 1.0f;
    s->prog = 64;
    s->ttf = -100.0f;
    s->alarm_age = -1.0f;
    s->outcome = LANDER_FLYING;
    s->cmd_thr = 0.5f;
}

bool lander_ttf_solve(float j, float a, float vg, float vdg,
                      float rg, float rdg, float *ttf)
{
    float t = *ttf;
    const float c1 = 6.0f * vg + 18.0f * vdg;
    const float c0 = 24.0f * (rdg - rg);
    for (int i = 0; i < 12; i++) {
        float f  = ((j * t + 6.0f * a) * t + c1) * t + c0;
        float df = (3.0f * j * t + 12.0f * a) * t + c1;
        if (fabsf(df) < 1e-6f) return false;
        float nt = t - f / df;
        if (fabsf(nt - t) < 0.01f) { t = nt; break; }
        t = nt;
    }
    if (!(t < 0.0f) || isnan(t)) return false;
    *ttf = t;
    return true;
}

static void thrust_from_acg(lander_t *s, float acg_x, float acg_h)
{
    float tx = acg_x, th = acg_h + LANDER_G_MOON;   // ACG minus gravity
    if (th < 0.1f) th = 0.1f;                       // never command thrust down
    s->cmd_pitch = clampf(atan2f(tx, th), -LANDER_PITCH_MAX, LANDER_PITCH_MAX);
    s->cmd_thr   = clampf(sqrtf(tx * tx + th * th) / LANDER_AMAX, LANDER_THR_MIN, 1.0f);
}

void lander_guidance(lander_t *s, float dt)
{
    if (s->outcome != LANDER_FLYING) return;
    // BAILOUT: the restart after the 1202 drops guidance passes.
    if (s->alarm_age >= 0.0f && s->alarm_age < LANDER_BAILOUT_S) {
        s->guid_valid = false;
        return;
    }
    if (s->prog == 64) {
        s->ttf += dt;                                       // TTFINCR
        float ttf = s->ttf;
        bool ok = lander_ttf_solve(JDG_X, ADG_X, s->vx, VDG_X, s->x, RDG_X, &ttf);
        if (ok) s->ttf = ttf;
        if (!ok || s->ttf > -4.0f || s->h < LANDER_P66_H) {
            s->prog = 66;                                   // into rate-of-descent
        } else {
            float T = s->ttf, T2 = T * T;
            float acg_x = ADG_X + 6.0f * (VDG_X + s->vx) / T + 12.0f * (RDG_X - s->x) / T2;
            float acg_h = ADG_H + 6.0f * (VDG_H + s->vh) / T + 12.0f * (RDG_H - s->h) / T2;
            thrust_from_acg(s, acg_x, acg_h);
            s->guid_valid = true;
            return;
        }
    }
    // P66: descend at a rate that shrinks with altitude, null horizontal speed.
    float vdgvert = -clampf(s->h * 0.08f, 0.3f, 3.0f);
    float acg_h = (vdgvert - s->vh) / TAUVERT;
    float acg_x = -s->vx / TAUHORIZ;
    thrust_from_acg(s, acg_x, acg_h);
    s->guid_valid = true;
}

void lander_step(lander_t *s, float dt, float thr_target, float pitch_target)
{
    if (s->outcome != LANDER_FLYING) return;
    s->t += dt;
    if (s->alarm_age >= 0.0f) s->alarm_age += dt;

    // Bottom-heavy: with the throttle at minimum, small pitch commands are
    // ignored and the LM rights itself (larger ones are reduced by the deadzone).
    if (thr_target <= LANDER_THR_MIN) pitch_target = lander_self_right(pitch_target);

    // Actuators: throttle lag, attitude rate limit, no thrust without fuel.
    thr_target = clampf(thr_target, LANDER_THR_MIN, 1.0f);
    s->thr += (thr_target - s->thr) * (dt / (THR_TAU + dt));
    float dp = clampf(clampf(pitch_target, -LANDER_PITCH_MAX, LANDER_PITCH_MAX) - s->pitch,
                      -PITCH_RATE * dt, PITCH_RATE * dt);
    s->pitch += dp;

    float thr = s->fuel > 0.0f ? s->thr : 0.0f;
    s->fuel -= thr * dt / LANDER_BURN_S;
    if (s->fuel < 0.0f) s->fuel = 0.0f;

    float a = thr * LANDER_AMAX;
    float ax = a * sinf(s->pitch);
    float ah = a * cosf(s->pitch) - LANDER_G_MOON;
    s->vx += ax * dt;  s->vh += ah * dt;
    s->x  += s->vx * dt; s->h += s->vh * dt;

    if (!s->alarm_fired && s->h < LANDER_ALARM_H) {   // program alarm 1202
        s->alarm_fired = true;
        s->alarm_age = 0.0f;
    }
    if (s->h <= 0.0f) {
        s->h = 0.0f;
        s->td_vh = s->vh; s->td_vx = s->vx; s->td_pitch = s->pitch; s->td_x = s->x;
        bool soft = s->vh >= -2.0f && fabsf(s->vx) <= 1.2f && fabsf(s->pitch) <= 0.21f;
        s->outcome = soft ? LANDER_LANDED : LANDER_CRASHED;
        s->vh = s->vx = 0.0f;
    }
}

bool lander_alarm_active(const lander_t *s)
{
    return s->alarm_age >= 0.0f && s->alarm_age < LANDER_ALARM_S;
}
