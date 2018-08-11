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

void tfrPathGenerator::PathGen(float x_mea, float y_mea, float x_dot_mea, float y_dot_mea, float &fwd_speed_demand)
{
    // stop just before reaching point
    // TODO - proper decel algorithm with park brake logic.
    if (sqrtf(x_ref*x_ref+y_ref*y_ref) < 1.0f) {
        fwd_speed_demand = 0.0f;
    }

    // define reciprocal unit vector
    float a_vec_x = -cosf(theta_ref);
    float a_vec_y = -sinf(theta_ref);

    // Set reference position along demanded path to be parallel to ground vehicle
    x_ref = a_vec_x * x_mea;
    y_ref = a_vec_y * y_mea;

    // set reference velocity to projection of ground vehicle velocity along demanded path
    x_ref_dot = -a_vec_x * x_dot_mea;
    y_ref_dot = -a_vec_y * y_dot_mea;
}

void tfrPathGenerator::GetRefs(tfr_pg_refs_t &refs)
{
    refs.x_ref         = x_ref;
    refs.y_ref         = y_ref;
    refs.x_ref_dot     = x_ref_dot;
    refs.y_ref_dot     = y_ref_dot;
    refs.theta_ref_dot = theta_ref_dot;
}
