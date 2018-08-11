/* tfrPathController

Terrafugia Rover path controller

Author: Ming Xin / Paul Riseborough / Vic Sperry

*/

typedef struct {
    float kp_y;
    float ki_y;
    float kd_y;
    float i_max;
    float kp_v;
    float v_max;
} tfr_pc_initblk_t;

class tfrPathController {
public:
    tfrPathController(tfr_pc_initblk_t &initblk);
    void PathControl(float x_ref, float y_ref, float x_dot_ref,
                     float y_dot_ref, float yaw_rate_ref, float x_mea,
                     float y_mea, float x_dot_mea, float y_dot_mea, float yaw_mea);
    float SteeringAngle();
    float SpeedDemand();
private:
    float kp_y;
    float ki_y;
    float kd_y;
    float i_max;
    float kp_v;
    float v_max;
    float integ_state_1;

    float x_err;
    float y_err;
    float x_dot_err;
    float y_dot_err;
    float cos_yaw;
    float sin_yaw;
    float cross_track_error;
    float cross_track_rate_error;
    float L;    // wheelbase length
    float speed;
    float along_track_error;
    float y_accln;

    float steering_angle;
    float speed_demand;
    bool initialized;
    uint32_t last_micros;
};
