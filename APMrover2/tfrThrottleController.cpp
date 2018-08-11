/* tfrThrottleController

Terrafugia Rover throttle control by using PID algorithm class definition

Author: Ming Xin / Paul Riseborough / Vic Sperry
Created Time: @1:59 PM, Aug 03, 2018;

*/

// TODO: comment out
#define TFR_THR_DEBUG 1

#ifdef TFR_THR_DEBUG
#include "Rover.h"  // for access to gcs().send_text()
#endif

#include "tfrThrottleController.h"

tfrThrottleController::tfrThrottleController() {
    struct _pid iPid = {0.0, }; // init to all 0.0
    PID_init(iPid);
}

tfrThrottleController::tfrThrottleController(struct _pid &iPid)
{
    PID_init(iPid);
}

void tfrThrottleController::PID_init(struct _pid &iPid)
{
    pid.set_speed        = iPid.set_speed;
    pid.actual_speed     = iPid.actual_speed;
    pid.speed_error      = iPid.speed_error;
    pid.speed_error_last = iPid.speed_error_last;
    pid.thrust           = iPid.thrust;
    pid.integral         = iPid.integral;
    pid.diff_error       = iPid.diff_error;
    pid.Kp               = iPid.Kp;
    pid.Ki               = iPid.Ki;
    pid.Kd               = iPid.Kd;
	pid.Kff              = iPid.Kff;
	pid.ilim             = iPid.ilim;

#ifdef TFR_THR_DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "\nThrottle Controller initialized with:");
    gcs().send_text(MAV_SEVERITY_INFO, "  Kp   = %f", iPid.Kp);
    gcs().send_text(MAV_SEVERITY_INFO, "  Ki   = %f", iPid.Ki);
    gcs().send_text(MAV_SEVERITY_INFO, "  Kd   = %f", iPid.Kd);
    gcs().send_text(MAV_SEVERITY_INFO, "  Kff  = %f", iPid.Kff);
    gcs().send_text(MAV_SEVERITY_INFO, "  ilim = %f", iPid.ilim);
#endif
}

// given_speed is the speed setpoint
// measured_speed is the longitudinal, measured speed
float tfrThrottleController::PID_speed_control(float given_speed, float measured_speed)
{
    pid.set_speed   = given_speed;
    pid.speed_error = pid.set_speed - measured_speed;         // get speed tracking error, penalized by proportional gain, Kp
    pid.integral    = pid.Ki*(pid.speed_error + pid.integral);// Intergal error: accurmulated error should be penalized by integral gain, Ki
    pid.integral    = constrain_float(pid.integral, -pid.ilim, pid.ilim);
    pid.diff_error  = pid.speed_error - pid.speed_error_last;
    
    // control law:
    pid.thrust = pid.Kp*pid.speed_error + pid.integral + pid.Kd*pid.diff_error + pid.Kff*pid.set_speed;

    // update speed_error
    pid.speed_error_last = pid.speed_error;
    pid.actual_speed    = pid.thrust * 1.0f;    // new set point
    
    return pid.actual_speed;
}
