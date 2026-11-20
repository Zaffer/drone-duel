// drone-duels: the quad's plant in the page. SkyFlow-Dynamics' generated C backend (sfd.c,
// written by `python -m skyflow_dynamics.backends.c`; tests/test_web_sim.py holds it to the
// JAX backend the game's own physics runs) behind the same module as the firmware
// (bfweb.c), so one instance is one quad: its flight controller and its airframe.
//
// The page's worker (web/src/sim/quad.ts) owns the model around it (the ESCs, the pack, the
// wind, the ground), as drone_duels.physics.quad.model does for the game's server. It writes
// the state, inputs and parameters into the arrays here and reads them back: SkyFlow's flat
// layouts (sfd.h), doubles.

#include "sfd.h"

#define EXPORT(name) __attribute__((used, export_name(#name)))

static double state[SFD_NS];
static double inputs[SFD_NU];
static double params[SFD_NP];
static double imu[6];

EXPORT(plant_state) double *plant_state(void)
{
    return state;
}

EXPORT(plant_inputs) double *plant_inputs(void)
{
    return inputs;
}

EXPORT(plant_params) double *plant_params(void)
{
    return params;
}

EXPORT(plant_imu) double *plant_imu(void)
{
    return imu;
}

EXPORT(plant_sizes) int plant_sizes(int which)
{
    switch (which) {
    case 0: return SFD_N;
    case 1: return SFD_NS;
    case 2: return SFD_NU;
    case 3: return SFD_NP;
    default: return 0;
    }
}

// `steps` exact-exponential steps of `dt` s with the inputs held, the quaternion renormalised
// and the rotor speeds clipped to [0, rotor_max] after each (the JAX backend's make_rollout,
// as drone_duels.physics.quad.plant builds it); then the exact IMU at the end (specific force,
// rates: body forward, left, up), at the centre of mass.
EXPORT(plant_advance) void plant_advance(double dt, int steps, double rotor_max)
{
    static const double zero[3] = {0.0, 0.0, 0.0};
    static const double eye[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    sfd_advance(state, inputs, params, dt, steps, 0.0, rotor_max);
    sfd_imu(state, inputs, params, zero, eye, imu);
}

EXPORT(plant_throttle_to_speed) double plant_throttle_to_speed(double u, double w_min, double w_max, double k)
{
    return sfd_throttle_to_speed(u, w_min, w_max, k);
}
