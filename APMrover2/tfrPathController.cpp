/* tfrPathController

Terrafugia Rover path controller

Author: Ming Xin / Paul Riseborough / Vic Sperry

*/  

// TODO: comment out
#define TFR_PATH_CTL_DEBUG 1

#ifdef TFR_PATH_CTL_DEBUG
#include "Rover.h"  // for access to gcs().send_text()
#endif
#include "tfrPathController.h"

tfrPathController::tfrPathController(tfr_pc_initblk_t &initblk) {

    // tunable parameters
    kp_y  = initblk.kp_y;
    ki_y  = initblk.ki_y;
    kd_y  = initblk.kd_y;
    i_max = initblk.i_max;
    kp_v  = initblk.kp_v;
    v_max = initblk.v_max;

    // steering law integrator state
    integ_state_1 = 0;

    initialized = false;

#ifdef TFR_PATH_CTL_DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "\ntfrPathController initialized with:");
    gcs().send_text(MAV_SEVERITY_INFO, "  kp_y  = %f", kp_y);
    gcs().send_text(MAV_SEVERITY_INFO, "  ki_y  = %f", ki_y);
    gcs().send_text(MAV_SEVERITY_INFO, "  kd_y  = %f", kd_y);
    gcs().send_text(MAV_SEVERITY_INFO, "  i_max = %f", i_max);
    gcs().send_text(MAV_SEVERITY_INFO, "  kp_v  = %f", kp_v);
    gcs().send_text(MAV_SEVERITY_INFO, "  v_max = %f", v_max);
#endif
}

void tfrPathController::PathControl(
    float x_ref,
    float y_ref,
    float x_dot_ref,
    float y_dot_ref,
    float yaw_rate_ref,
    float x_mea,
    float y_mea,
    float x_dot_mea,
    float y_dot_mea,
    float yaw_mea)
{
    uint32_t micros;
    float dt;

    // theta_ref_dot and theta_ref
    if( ! initialized ) {
        // we're supposed to be running at 50 Hz. First time through, it'll be perfect.
        // Unsigned arithmetic will handle rollover. Trust me.
        last_micros = AP_HAL::micros() - 20000;
    }

    // get actual dt, which should be about 20 ms
    micros = AP_HAL::micros();
    dt = ((float)(micros - last_micros) / ((float)AP_USEC_PER_SEC));   // dt in seconds
    last_micros = micros;

    // calculate component of position and velocity error perpendicular to reference trajectory
    x_err = x_ref - x_mea;
    y_err = y_ref - y_mea;
    x_dot_err = x_dot_ref - x_dot_mea;
    y_dot_err = y_dot_ref - y_dot_mea;
    cos_yaw = cosf(yaw_mea);
    sin_yaw = sinf(yaw_mea);
    cross_track_error = cos_yaw * y_err - sin_yaw * x_err;
    cross_track_rate_error = cos_yaw * y_dot_err - sin_yaw * x_dot_err;

    // steer back to track using PID law to calculate lateral acceleration
    speed = x_dot_mea * cos_yaw + y_dot_mea * sin_yaw;
    speed = MAX(speed,0.1f);
    float cross_track_err_lim = 0.5f*speed*kd_y/kp_y;
    cross_track_error = constrain_float(cross_track_error,-cross_track_err_lim,cross_track_err_lim);
    integ_state_1 = integ_state_1 + dt * cross_track_error * ki_y;
    integ_state_1 = constrain_float(integ_state_1, -i_max, i_max);
    y_accln = kp_y * cross_track_error + integ_state_1 + kd_y * cross_track_rate_error;

    // Convert lateral acceleration to steering angle
    L = 0.4286f;
    steering_angle = atanf(y_accln * L / speed*speed);

    // add feed forward term based on predicted path curvature
    steering_angle = steering_angle + atanf(yaw_rate_ref * L / speed);

    // calculate speed demand
    speed_demand = sqrtf(x_dot_ref * x_dot_ref + y_dot_ref * y_dot_ref);
    along_track_error = cos_yaw * x_err + sin_yaw * y_err;
    speed_demand = speed_demand + kp_v * along_track_error;
    speed_demand = MIN(speed_demand, v_max);

    initialized = true;
}

float tfrPathController::SteeringAngle()
{
    return steering_angle;
}

float tfrPathController::SpeedDemand()
{
    return speed_demand;
}
