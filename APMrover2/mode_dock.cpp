#include "mode.h"
#include "Rover.h"
#include "tfrThrottleController.h"
#include "tfrPathGenerator.h"
#include "tfrPathController.h"

static tfrThrottleController *pSpeedPid = NULL;
static tfrPathGenerator *pPathGen = NULL;
static tfrPathController *pPathCtl = NULL;

void docking_data(float &x_ref, float &y_ref, float &x_mea, float &y_mea, float &spd_dem)
{
    tfr_pg_refs_t refs;
    pPathGen->GetRefs(refs);
    x_ref = refs.x_ref;
    y_ref = refs.y_ref;
    pPathGen->GetSpdDem(spd_dem);
    // position of Rover in local NED reference frame
    Vector2f posNE = {0.0, 0.0};
    if( ! AP::ahrs().get_relative_position_NE_home(posNE) ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "get_relative_position_NE_home() failed");
        return;
    }
    x_mea = posNE[0];
    y_mea = posNE[1];
}

//
// Action to take upon entering DOCK mode
//
bool ModeDOCK::_enter()
{
    if( ! init_path_generator() )
    {
        return false;
    }

    if( ! init_path_controller() )
    {
        delete pPathGen;
        pPathGen = NULL;

        return false;
    }

    if( ! init_throttle_controller() )
    {
        delete pPathCtl;
        pPathCtl = NULL;

        delete pPathGen;
        pPathGen = NULL;

        return false;
    }

    return true;
}

bool ModeDOCK::init_path_generator()
{
    // If there is already a path generator (shouldn't be), delete it
    if( pPathGen != NULL ) {
        delete pPathGen;
        pPathGen = NULL;
    }

    // These will ultimately come from the path planner
    pathgen_kappa = g2.tfr_pathgen_kappa;
    pathgen_fwd_speed_demand = g2.tfr_pathgen_fwd_speed_demand;

    // position of Rover in local NED refererence frame
    Vector2f posNE = {0.0, 0.0};

    if( ! AP::ahrs().get_relative_position_NE_home(posNE) ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "get_relative_position_NE_home() failed");
        return false;
    }

    // angle from Local NED North axis to rover X axis, radians
    float yaw = AP::ahrs().yaw;

	// create a PathGenerator object
    pPathGen = new tfrPathGenerator(posNE[0], posNE[1], yaw);
    if( pPathGen == NULL ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Could not allocate tfrPathGenerator object");
        return false;
    }

    return true;
}

bool ModeDOCK::init_path_controller()
{
    // initialize the initblk to all zeros.
    tfr_pc_initblk_t initblk = {0.0, };

    // If there is already a path controller (shouldn't be), delete it
    if( pPathCtl != NULL ) {
        delete pPathCtl;
        pPathCtl = NULL;
    }

    // fill in the tunable parameters
    initblk.kp_y  = g2.tfr_pathctl_kp_y;
    initblk.ki_y  = g2.tfr_pathctl_ki_y;
    initblk.kd_y  = g2.tfr_pathctl_kd_y;
    initblk.i_max = g2.tfr_pathctl_i_max;
    initblk.kp_v  = g2.tfr_pathctl_kp_v;
    initblk.v_max = g2.tfr_pathctl_v_max;

	// create a PathController object
    pPathCtl = new tfrPathController(initblk);
    if( pPathCtl == NULL ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Could not allocate tfrPathController object");
        return false;
    }

    return true;
}

bool ModeDOCK::init_throttle_controller()
{
    // init to all 0.0
    struct _pid iPid = {0.0, };
    
    speed_setpoint = g2.tfr_speed_setpoint_default;
    throttle_scale = g2.tfr_throttle_scale;
    //gcs().send_text(MAV_SEVERITY_INFO, "speed_setpoint = %f m/s", speed_setpoint);
    //gcs().send_text(MAV_SEVERITY_INFO, "throttle_scale = %f", throttle_scale);

    // override the ones we care about
    iPid.Kp   = g2.tfr_throttle_kp;
    iPid.Ki   = g2.tfr_throttle_ki;
    iPid.Kd   = g2.tfr_throttle_kd;
	iPid.Kff  = g2.tfr_throttle_kff;
	iPid.ilim = g2.tfr_throttle_ilim;

    docking = true;
    down_counter = 0;

	// If there is already a speed object (shouldn't be), delete it
    if( pSpeedPid != NULL ) {
        delete pSpeedPid;
        pSpeedPid = NULL;
    }

	// create an object Pid -- constructor will call PID_init()
    pSpeedPid = new tfrThrottleController(iPid);
    if( pSpeedPid == NULL ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Could not allocate Rover speed controller object");
        return false;
    }

    //gcs().send_text(MAV_SEVERITY_INFO, "Created and initialized Rover speed controller object");
    return true;
}

void ModeDOCK::path_generator()
{
    if( pPathGen ) {
        // position of Rover in local NED reference frame
        Vector2f posNE = {0.0, 0.0};

        // Rover measured velocity in NED coordinate frame, m/s
        Vector3f velNED = {0.0, 0.0, 0.0};

        if( ! AP::ahrs().get_relative_position_NE_home(posNE) ) {
            gcs().send_text(MAV_SEVERITY_ERROR, "get_relative_position_NE_home() failed");
            return;
        }

        if( ! AP::ahrs().get_velocity_NED(velNED) ) {
            gcs().send_text(MAV_SEVERITY_ERROR, "get_velocity_NED() failed");
            return;
        }
        pPathGen->PathGen(posNE[0], posNE[1], velNED[0], velNED[1], pathgen_fwd_speed_demand);
    }
}

void ModeDOCK::path_controller()
{
    if( pPathCtl && pPathGen ) {

        tfr_pg_refs_t refs;

        // position of Rover in local NED reference frame
        Vector2f posNE = {0.0, 0.0};

        // Rover measured velocity in NED coordinate frame, m/s
        Vector3f velNED = {0.0, 0.0, 0.0};

        // angle from Local NED North axis to rover X axis, radians
        float yaw_mea = AP::ahrs().yaw;

        pPathGen->GetRefs(refs);
        
        if( ! AP::ahrs().get_relative_position_NE_home(posNE) ) {
            gcs().send_text(MAV_SEVERITY_ERROR, "get_relative_position_NE_home() failed");
            return;
        }

        if( ! AP::ahrs().get_velocity_NED(velNED) ) {
            gcs().send_text(MAV_SEVERITY_ERROR, "get_velocity_NED() failed");
            return;
        }

        pPathCtl->PathControl(refs.x_ref, refs.y_ref, refs.x_ref_dot, refs.y_ref_dot,
                              refs.theta_ref_dot, posNE[0] /* x_mea */, posNE[1] /* y_mea */,
                              velNED[0] /* x_dot_mea */, velNED[1] /* y_dot_mea */, yaw_mea);
    }
}

void ModeDOCK::set_steering_output()
{
    if( pPathCtl == NULL ) {
        return;
    }

    // steering angle coming out of Path controller is in radians
    float steering_angle = pPathCtl->SteeringAngle();

    // Create a normalized steering input in the range -1..1.
    // This will need some tweaking, so connect it directly to a tunable parameter
    float steering = steering_angle * g2.tfr_steering_sf;    

    g2.motors.set_steering( steering * 4500.0, false);
}

//
// Given a rover velocity in NE frame (m/s), and vehicle yaw angle (radians),
// produce the rover speed in the direction of the body frame (x-direction).
// Yaw angle is in radians relative to N axis with CW being positive.
//
float ModeDOCK::rover_speed_x(Vector2f velNE, float yaw)
{
    // unit vector in body frame, x-direction
    Vector2f velx_unit = { cosf(yaw), sinf(yaw) };

    // return the dot product of the two vectors
    return velx_unit * velNE;
}

void ModeDOCK::throttle_controller()
{
    // throttle_norm is the normalized throttle as produced by the TTSPID
    // throttle_norm is in range -1.0 .. 1.0:
    // -1.0 => maximum reverse throttle
    //  0.0 => throttle off
    // +1.0 => maximum forward throttle
    float throttle_norm = 0.0;

    // +/- 100.0
    float throttle_out = 0.0;

    // actual speed in m/s in rover X direction (longitudinal to body frame)
    float speed_actual = 0.0;

    // angle from Local NED North axis to rover X axis, radians
    float yaw = AP::ahrs().yaw;

    // Rover measured velocity in NED coordinate frame, m/s
    Vector3f velNED = {0.0, 0.0, 0.0};

    if( ! AP::ahrs().get_velocity_NED(velNED) ) {
        gcs().send_text(MAV_SEVERITY_ERROR, "get_velocity_NED() failed");
        return;
    }

    // actual speed along longitudinal axis of rover
    speed_actual = rover_speed_x( { velNED[0], velNED[1] }, yaw );
        
    // Apply TTSPID (Terrafugia Trivial Speed PID)
    if( pSpeedPid ) {
        throttle_norm = pSpeedPid->PID_speed_control(pathgen_fwd_speed_demand, speed_actual);
    }

    // Convert throttle_norm in range -1.0..1.0 to a throttle_out value
    throttle_out = throttle_norm * throttle_scale;
    
    // constrain throttle_out
    throttle_out = constrain_float(throttle_out, -100.0f, 100.0f);

    // set Rover throttle PWM
    g2.motors.set_throttle(throttle_out);

    // TODO remove this
    // every second print out some information
    //if( down_counter == 0 ) {
    //    static uint32_t mic0 = AP_HAL::micros();
    //    uint32_t mic = AP_HAL::micros();
    //    uint32_t dt = mic - mic0;
    //    mic0 = mic;
    //    gcs().send_text(MAV_SEVERITY_INFO, "micros() = %u (0x%08x), dt = %u (0x%08x)", mic, mic, dt, dt);
    //    gcs().send_text(MAV_SEVERITY_INFO, "velNE         = {%f, %f}", velNED[0], velNED[1]);
    //    gcs().send_text(MAV_SEVERITY_INFO, "speed_actual  = %f", speed_actual);
    //    gcs().send_text(MAV_SEVERITY_INFO, "throttle_norm = %f", throttle_norm);
    //    gcs().send_text(MAV_SEVERITY_INFO, "throttle_out  = %f", throttle_out);
    //}
}

// Called at a 50 Hz rate
void ModeDOCK::update()
{
    path_generator();
    path_controller();
    set_steering_output();
    throttle_controller();

    // implement a coarse timer.
    if( down_counter > 0 ) {
        down_counter--;
    } else {
        down_counter = 50;
    }
}

void ModeDOCK::_exit()
{
    docking = false;
    down_counter = 0;

    if( pSpeedPid != NULL ) {
        delete pSpeedPid;
        pSpeedPid = NULL;
    }
    
    if( pPathCtl != NULL ) {
        delete pPathCtl;
        pPathCtl = NULL;
    }
    
    if( pPathGen != NULL ) {
        delete pPathGen;
        pPathGen = NULL;
    }
}
