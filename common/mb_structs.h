#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    float   wheelAngleR;
    float   wheelAngleL;
    float   prev_wheelAngleR;
    float   prev_wheelAngleL;
    float   gamma;
    float   shifted_gamma; //new
    float   prev_gamma;
    float   prev_unwr_gamma;
    float   prev_shifted;
    float   unwrapped_gamma;
    float   enc_gamma;

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needed
    float prev_theta;
    float prev_phi;
};

typedef struct mb_gains mb_gains_t;
struct mb_gains{
    float KPi;
    float KIi;
    float KDi;
    float Tfi;
    float gi;

    float KPo;
    float KIo;
    float KDo;
    float Tfo;
    float go;

    float KPs;
    float KIs;
    float KDs;
    float Tfs;
    float gs;

    float KPh;
    float KIh;
    float KDh;
    float Tfh;
    float gh;

    int is_inner;
    int is_heading;
    int bot_mode;
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{
    float rc_heading;
    float traj_heading;
    float phi_desired;
    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{
    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float delta_d;
    float fused_theta;
    float prev_fused_theta;
};

typedef struct mb_vel_prof mb_vel_prof_t;
struct mb_vel_prof{
    float v;
    float w;
    float acc;
};

mb_gains_t gains;

#endif
