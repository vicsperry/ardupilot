/* tfrPathGenerator

Terrafugia Rover path generator

Author: Ming Xin / Paul Riseborough / Vic Sperry

*/  
typedef struct {
    float x_ref;
    float y_ref;
    float theta_ref;
    float x_ref_dot;
    float y_ref_dot;
    float theta_ref_dot;
} tfr_pg_refs_t;

class tfrPathGenerator{
public:
    tfrPathGenerator(float x_mea, float y_mea, float theta_mea);
    void PathGen(float kappa, float fwd_speed_demand);
    void GetRefs(tfr_pg_refs_t &refs);
private:
    float integ_state_theta;
    float theta_dot_prev;
    float integ_state_x;
    float x_ref_dot_prev;
    float integ_state_y;
    float y_ref_dot_prev;
    float x_ref_dot;
    float theta_ref;
    float x_ref;
    float y_ref_dot;
    float y_ref;
    float theta_ref_dot;
    bool initialized;
    uint32_t last_micros;
    int down_counter;
};
