/* tfrPathGenerator

Terrafugia Rover path generator

Author: Ming Xin / Paul Riseborough / Vic Sperry

*/  

// TODO: comment out
#define TFR_PATH_GEN_DEBUG 1

#ifdef TFR_PATH_GEN_DEBUG
#include "Rover.h"  // for access to gcs().send_text()
#endif

#include "tfrPathGenerator.h"

tfrPathGenerator::tfrPathGenerator(float x_mea, float y_mea, float theta_mea) {
    integ_state_theta = radians(5.0f);
    float radius = sqrtf(x_mea*x_mea+y_mea*y_mea);
    integ_state_x     = -radius*cosf(integ_state_theta);
    integ_state_y     = -radius*sinf(integ_state_theta);;
    initialized       = false;
#ifdef TFR_PATH_GEN_DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "\ntfrPathGenerator initialized with:");
    gcs().send_text(MAV_SEVERITY_INFO, "  integ_state_theta = %f", integ_state_theta);
    gcs().send_text(MAV_SEVERITY_INFO, "  integ_state_x     = %f", integ_state_x);
    gcs().send_text(MAV_SEVERITY_INFO, "  integ_state_y     = %f", integ_state_y);
#endif
}

void tfrPathGenerator::PathGen(float kappa, float fwd_speed_demand)
{
    if (sqrtf(x_ref*x_ref+y_ref*y_ref) < 1.0f) {
        fwd_speed_demand = 0.0f;
    }
    uint32_t micros;
    float dt;

    // theta_ref_dot and theta_ref

    if( ! initialized ) {
        theta_dot_prev = kappa * fwd_speed_demand;

        // we're supposed to be running at 50 Hz. First time through, it'll be perfect.
        // Unsigned arithmetic will handle rollover. Trust me.
        last_micros = AP_HAL::micros() - 20000;
        down_counter = 20;
#ifdef TFR_PATH_GEN_DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "tfrPathGenerator doing secondary initialization");
        gcs().send_text(MAV_SEVERITY_INFO, "  theta_dot_prev = %f", theta_dot_prev);
#endif
    }

    // get actual dt, which should be about 20 ms
    micros = AP_HAL::micros();
    dt = ((float)(micros - last_micros) / ((float)AP_USEC_PER_SEC));   // dt in seconds
    last_micros = micros;

    theta_ref_dot = kappa * fwd_speed_demand;
    integ_state_theta = integ_state_theta + (0.5 * (theta_ref_dot + theta_dot_prev) * dt);
    theta_dot_prev = theta_ref_dot;
    theta_ref = integ_state_theta;

    // x_ref_dot and x_ref
    if( ! initialized ) {
        x_ref_dot_prev = fwd_speed_demand * cosf(theta_ref);
#ifdef TFR_PATH_GEN_DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "  x_ref_dot_prev = %f", x_ref_dot_prev);
#endif
    }

    x_ref_dot = fwd_speed_demand * cosf(theta_ref);
    integ_state_x = integ_state_x + (0.5 * (x_ref_dot + x_ref_dot_prev) * dt);
    x_ref_dot_prev = x_ref_dot;
    x_ref = integ_state_x;

    // y_ref_dot and y_ref
    if( ! initialized ) {
        y_ref_dot_prev = fwd_speed_demand * sinf(theta_ref);
#ifdef TFR_PATH_GEN_DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "  y_ref_dot_prev = %f", y_ref_dot_prev);
#endif
    }

    y_ref_dot = fwd_speed_demand * sinf(theta_ref);
    integ_state_y = integ_state_y + (0.5 * (y_ref_dot + y_ref_dot_prev) * dt);
    y_ref_dot_prev = y_ref_dot;
    y_ref = integ_state_y;

    initialized = true;
}

void tfrPathGenerator::GetRefs(tfr_pg_refs_t &refs)
{
    refs.x_ref         = x_ref;
    refs.y_ref         = y_ref;
    refs.x_ref_dot     = x_ref_dot;
    refs.y_ref_dot     = y_ref_dot;
    refs.theta_ref_dot = theta_ref_dot;
}
