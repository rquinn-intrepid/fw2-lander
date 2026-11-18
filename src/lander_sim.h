// lander_sim — pure lunar-descent model + Luminary 099 style landing guidance.
//
// No Pico SDK, no hardware: built into the app and into tests/ (fw test).
//
// Frame: 2-D vertical plane. x = downrange metres, landing site at x = 0,
// increasing toward (and past) the site; h = altitude metres. pitch = thrust
// tilt in radians, positive tips the thrust toward +x.
//
// Guidance follows LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (Luminary 099):
//   * APPRQUAD (P64): time-to-go TTF (negative, AGC convention) is advanced
//     by the pass interval (TTFINCR) and then refined by Newton iteration
//     (ROOTPSRS) on the downrange cubic
//         JDG*T^3 + 6*ADG*T^2 + (6*VG + 18*VDG)*T + 24*(RDG - RG) = 0
//     and the commanded acceleration is the published quadratic law
//         ACG = ADG + 6*(VDG + VG)/TTF + 12*(RDG - RG)/TTF^2
//   * VERTICAL (P66, rate of descent): ACG = (VDGVERT - VG) / TAUVERT.
// Thrust command = ACG - gravity, split into throttle and pitch.
#ifndef LANDER_SIM_H
#define LANDER_SIM_H

#include <stdbool.h>

#define LANDER_G_MOON      1.62f   // m/s^2
#define LANDER_AMAX        3.60f   // full-throttle thrust acceleration, m/s^2
#define LANDER_THR_MIN     0.10f   // DPS lower throttle limit
#define LANDER_BURN_S      85.0f   // seconds of fuel at full throttle
#define LANDER_PITCH_MAX   0.785f  // 45 degrees
#define LANDER_ALARM_H     609.6f  // 2000 ft: the Apollo 11 1202 altitude
#define LANDER_BAILOUT_S   0.6f    // guidance frozen while the AGC restarts
#define LANDER_ALARM_S     4.0f    // V05N09 alarm display time
#define LANDER_P66_H       45.7f   // 150 ft: switch to rate-of-descent

typedef enum { LANDER_FLYING = 0, LANDER_LANDED, LANDER_CRASHED } lander_outcome_t;

typedef struct {
    // state
    float x, h, vx, vh;   // m, m, m/s, m/s
    float thr;            // actual throttle 0..1 (lags the command)
    float pitch;          // actual thrust tilt, rad
    float fuel;           // 1.0 full .. 0.0 empty
    float t;              // mission elapsed, s
    // guidance
    int   prog;           // 64 or 66
    float ttf;            // time to go, s (negative)
    float cmd_thr;        // guidance throttle 0..1
    float cmd_pitch;      // guidance pitch, rad
    bool  guid_valid;     // false while frozen / before first pass
    // 1202 alarm
    bool  alarm_fired;    // it happens once
    float alarm_age;      // seconds since it fired; < 0 = never
    // touchdown
    lander_outcome_t outcome;
    float td_vh, td_vx, td_pitch, td_x;
} lander_t;

void  lander_reset(lander_t *s);
// Newton refinement of the TTF cubic. Returns false if it did not converge
// to a negative root (the AGC would raise alarm 1406 here).
bool  lander_ttf_solve(float jdg, float adg, float vg, float vdg,
                       float rg, float rdg, float *ttf_inout);
// One guidance pass: fills cmd_thr / cmd_pitch / ttf / prog.
void  lander_guidance(lander_t *s, float dt);
// Advance physics. thr_target 0..1 (clamped up to THR_MIN), pitch_target rad.
void  lander_step(lander_t *s, float dt, float thr_target, float pitch_target);
// True while the alarm display should be up (alarm fired, < ALARM_S old).
bool  lander_alarm_active(const lander_t *s);

#endif
