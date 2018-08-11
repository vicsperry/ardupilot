/* tfrThrottleController

Terrafugia Rover throttle control by using PID algorithm class definition

Author: Ming Xin / Paul Riseborough / Vic Sperry
Created Time: @1:59 PM, Aug 03, 2018;

*/  

typedef struct _pid{
    float set_speed;        // reference vehicle speed
    float actual_speed;     // current vehicle speed
    float speed_error;      // vehicle speed error
    float speed_error_last; // speed error in the last sampling time step
    float Kp;               // P gain
    float Ki;               // I gain
    float Kd;               // D gain
	float Kff;			    // Feed Forward Gain
	float ilim;			    // limit applied to accumulated integrator control action
    float integral;         // accumulated integrator control action
    float diff_error;       // differential error
    float thrust;           // control input for speed thrust
} Pid;

class tfrThrottleController{
public:
    tfrThrottleController();   // constructor
    tfrThrottleController(struct _pid &iPid);   // constructor with initializer
    void PID_init(struct _pid &iPid);
    float PID_speed_control(float given_speed, float measured_speed);
private:
    Pid pid;    
};
