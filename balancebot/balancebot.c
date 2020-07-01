/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
*
*******************************************************************************/
#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>

#include "balancebot.h"

int bot_mode;
float start_x;
float start_y;
float start_angle;

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
  // make sure another instance isn't running
  // if return value is -3 then a background process is running with
  // higher privaledges and we couldn't kill it, in which case we should
  // not continue or there may be hardware conflicts. If it returned -4
  // then there was an invalid argument that needs to be fixed.
  if(rc_kill_existing_process(2.0)<-2) return -1;

  // start signal handler so we can exit cleanly
  if(rc_enable_signal_handler()==-1){
    fprintf(stderr,"ERROR: failed to start signal handler\n");
    return -1;
  }

  if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
    fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
    return -1;
  }

  // initialize enocders
  if(rc_encoder_eqep_init()==-1){
    fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
    return -1;
  }

  // initialize adc
  if(rc_adc_init()==-1){
    fprintf(stderr, "ERROR: failed to initialize adc\n");
    return -1;
  }

  //initialize dsm
  if(rc_dsm_init()==-1){
    fprintf(stderr,"failed to start initialize DSM\n");
    return -1;
  }

  printf("initializing xbee... \n");
  //initalize XBee Radio
  int baudrate = BAUDRATE;
  if(XBEE_init(baudrate)==-1){
    fprintf(stderr,"Error initializing XBee\n");
    return -1;
  };

  // make PID file to indicate your project is running
  // due to the check made on the call to rc_kill_existing_process() above
  // we can be fairly confident there is no PID file already and we can
  // make our own safely.
  rc_make_pid_file();

  // start printf_thread if running from a terminal
  // if it was started as a background process then don't bother
  printf("starting print thread... \n");
  pthread_t  printf_thread;
  rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

  // start control thread
  printf("starting setpoint thread... \n");
  pthread_t  setpoint_control_thread;
  rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


  // TODO: start motion capture message recieve thread

  // set up IMU configuration
  printf("initializing imu... \n");
  // set up mpu configuration
  rc_mpu_config_t mpu_config = rc_mpu_default_config();
  mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
  mpu_config.orient = ORIENTATION_Z_DOWN;
  if(!rc_mpu_is_gyro_calibrated()){
    printf("Gyro not calibrated, automatically starting calibration routine\n");
    printf("Let your MiP sit still on a firm surface\n");
    rc_mpu_calibrate_gyro_routine(mpu_config);
  }

  // now set up the imu for dmp interrupt operation
  if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
    printf("rc_mpu_initialize_failed\n");
    return -1;
  }

  //rc_nanosleep(5E9); // wait for imu to stabilize

  //initialize state mutex
  pthread_mutex_init(&state_mutex, NULL);
  pthread_mutex_init(&setpoint_mutex, NULL);

  //attach controller function to IMU interrupt
  printf("initializing controller...\n");
  mb_setpoints.manual_ctl = 0;
  mb_controller_init();
  bot_mode = gains.bot_mode;
  printf("initializing motors...\n");
  mb_motor_init();

  printf("resetting encoders...\n");
  rc_encoder_eqep_write(1, 0);
  rc_encoder_eqep_write(2, 0);

  printf("initializing odometry...\n");
  mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

  printf("attaching imu interupt...\n");
  //Initializing state variables.
  mb_state.prev_theta = 0;
  mb_state.prev_phi = 0;
  rc_mpu_set_dmp_callback(&balancebot_controller);

  printf("we are running!!!...\n");
  // done initializing so set state to RUNNING
  rc_set_state(RUNNING);

  // Keep looping until state changes to EXITING
  while(rc_get_state()!=EXITING){

    // all the balancing is handled in the imu interupt function
    // other functions are handled in other threads
    // there is no need to do anything here but sleep
    // always sleep at some point
    rc_nanosleep(1E9);
  }

  // exit cleanly
  rc_mpu_power_off();
  mb_motor_cleanup();
  rc_led_cleanup();
  rc_encoder_eqep_cleanup();
  rc_remove_pid_file(); // remove pid file LAST
  return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
*
*
*******************************************************************************/
void balancebot_controller(){

  //lock state mutex
  pthread_mutex_lock(&state_mutex);

  // Read IMU
  mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
  mb_state.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];
  //--New code--//
  // if (mb_state.gamma < 0) {
  //     mb_state.shifted_gamma = 6.28 + mb_state.gamma;
  // } else {
  //     mb_state.shifted_gamma = mb_state.gamma;
  // }
  //--New code--//
  mb_state.enc_gamma = (mb_state.wheelAngleR - mb_state.wheelAngleL) * (WHEEL_DIAMETER / 2) / WHEEL_BASE; // For RC steering
  // Read encoders
  mb_state.left_encoder = rc_encoder_eqep_read(1);
  mb_state.right_encoder = rc_encoder_eqep_read(2);

  //Calculate wheel angles
  mb_state.wheelAngleR = (mb_state.right_encoder * 2.0 * M_PI) \
  /(ENC_2_POL * GEAR_RATIO * ENCODER_RES);
  mb_state.wheelAngleL = (mb_state.left_encoder * 2.0 * M_PI) \
  /(ENC_1_POL * GEAR_RATIO * ENCODER_RES);
  mb_state.phi = (mb_state.wheelAngleL + mb_state.wheelAngleR) / 2 + mb_state.theta;

  // Update odometry
  mb_odometry_update(&mb_odometry, &mb_state);

  // Calculate controller outputs
  if (mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry)) {
    printf("Failed to update controller outputs\n");
  }

  //send output to motors.
  if(!mb_setpoints.manual_ctl){
    if (bot_mode == 1) {
      drag_traj();
    }
    if (bot_mode == 2) {
      square_traj();
    }
    mb_motor_set(1, mb_state.left_cmd);
    mb_motor_set(2, mb_state.right_cmd);
  }
  if(mb_setpoints.manual_ctl){
    bot_mode = 3;
    float steering_input = mb_steer_controller(&mb_setpoints, &mb_state); // uses rc_heading and gamma (gyro)
    float rc_left_cmd = mb_state.left_cmd - MOT_1_POL * steering_input;
    float rc_right_cmd = mb_state.right_cmd + MOT_2_POL * steering_input;
    mb_motor_set(1, rc_left_cmd);
    mb_motor_set(2, rc_right_cmd);
  }

  /*
  XBEE_getData();
  double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
  double tb_array[3] = {0, 0, 0};
  rc_quaternion_to_tb_array(q_array, tb_array);
  mb_state.opti_x = xbeeMsg.x;
  mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
  mb_state.opti_roll = tb_array[0];
  mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
  mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
  */

  //Update previous values
  mb_state.prev_wheelAngleL = mb_state.wheelAngleL;
  mb_state.prev_wheelAngleR = mb_state.wheelAngleR;
  //unlock state mutex
  pthread_mutex_unlock(&state_mutex);
}

void drag_traj() {
  float x = mb_odometry.x;
  //v(x) method
  /*
  if (x <= 0.3) {
  mb_vel_prof.v = 2.66*x;
}
if (x > 0.3 && x < 0.8) {
mb_vel_prof.v = 0.8;
}
if (x >= 0.8 && x < 1.1) {
mb_vel_prof.v = -2.66*x + 2.933;
}
if (x == 1.1) {
mb_vel_prof.v = 0;
}
mb_vel_prof.w = 0;
*/

//V(t) method
float interval = 0.1;
if (x <= 0 + interval) {
  mb_vel_prof.acc = 6;
}
if (x >= 0.3 - interval && x <= 0.3 + interval) {
  mb_vel_prof.acc = 0;
}
if (x >= 0.8 - interval && x <= 0.8 + interval) {
  mb_vel_prof.acc = -6;
}
mb_vel_prof.v += mb_vel_prof.acc * DT;
if (mb_vel_prof.v < 0) {
  mb_vel_prof.v = 0;
}
if (x >= 1.1 - interval && x <= 1.1 + interval) {
  mb_vel_prof.v = 0;
}
mb_vel_prof.w = 0;
}

/*
bool is_start_turn() {
float r = 0.2;  // turn radius
float r_interval = 0.2;
float heading_interval = 0.2;
float x = mb_odometry.x;
float y = mb_odometry.y;
float h = mb_setpoints.traj_heading;
// if ((fabs(x - 0) < interval) && (fabs(y - 0) < interval)) {
//    return true;
// }
if ((fabs(x - (1 - r)) < r_interval) && (fabs(y - 0) < r_interval) && h >= 0 - heading_interval && h <= 0 + heading_interval) {
printf("Entered turn1 @ x:%f | y:%f\n", x, y);
return true;
}
if ((fabs(x - 1) < r_interval) && (fabs(y - (r - 1)) < r_interval) && h >= M_PI/2 - heading_interval && h <= M_PI/2 + heading_interval) {
printf("Entered turn2 @ x:%f | y:%f\n", x, y);
return true;
}
if ((fabs(x - r) < r_interval) && (fabs(y - (-1)) < r_interval) && h >= M_PI - heading_interval && h <= M_PI + heading_interval) {
printf("Entered turn3 @ x:%f | y:%f\n", x, y);
turn_flag = 0;
return true;
}
if ((fabs(x - 0) < r_interval) && (fabs(y - (-r)) < r_interval) && h >= -M_PI/2 - heading_interval && h <= -M_PI/2 + heading_interval) {
if (turn_flag == 1) {
return false;
}
printf("Entered turn4 @ x:%f | y:%f\n", x, y);
return true;
}
return false;
}
*/

void square_traj() {
  mb_vel_prof.v = 1.5;
  if (sqrt (pow((mb_odometry.x - start_x),2) + pow((mb_odometry.y - start_y),2)) > 0.9) {
    if (mb_state.unwrapped_gamma - start_angle < M_PI / 2) {
      mb_vel_prof.w = 0.3;
      mb_vel_prof.v = 0.5;
    } else {
      mb_vel_prof.w = 0;
      mb_vel_prof.v = 1.5;
      start_angle = mb_state.unwrapped_gamma;
      start_x = mb_odometry.x;
      start_y = mb_odometry.y;
    }
  }
}

/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

  while(1){
    // //initialize traj_heading to zero just for safety.
    // mb_setpoints.traj_heading = 0;

    //get setpoints from Remote Control if it is running.
    if(rc_dsm_is_new_data()){
      // TODO: Handle the DSM data from the Spektrum radio reciever
      // You may should implement switching between manual and autonomous mode
      // using channel 5 of the DSM data.
      // Read normalized (+-1) inputs from RC radio stick and multiply by
      // polarity setting so positive stick means positive setpoint
      mb_setpoints.manual_ctl  = (int)rc_dsm_ch_normalized(DSM_MAN_CH);
      if (mb_setpoints.manual_ctl) {
        double turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
        double drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH) * DSM_DRIVE_POL;
        // saturate the inputs to avoid possible erratic behavior
        rc_saturate_double(&drive_stick,-1,1);
        rc_saturate_double(&turn_stick,-1,1);
        // use a small deadzone to prevent slow drifts in position
        if(fabs(drive_stick)<FWD_VEL_SENSITIVITY) drive_stick = 0.0;
        if(fabs(turn_stick)<TURN_VEL_SENSITIVITY)  turn_stick  = 0.0;
        // translate normalized user input to real setpoint values
        mb_setpoints.fwd_velocity = DRIVE_RATE * drive_stick;
        mb_setpoints.turn_velocity = TURN_RATE * turn_stick;
        if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi_desired += mb_setpoints.fwd_velocity * DR_GEAR; // may need to initialize fwd_velocity to 0
        if(fabs(mb_setpoints.turn_velocity)> 0.0001) mb_setpoints.rc_heading += mb_setpoints.turn_velocity * DT;
      } else {

      }
    }
    rc_nanosleep(1E9 / RC_CTL_HZ);
    if (bot_mode == 0) {	// just balance
      mb_setpoints.phi_desired = 0;
      mb_setpoints.traj_heading = 0;
    }
    if (bot_mode == 1) {	// drag race
      drag_traj();
      if(fabs(mb_vel_prof.v) > 0.001) mb_setpoints.phi_desired += mb_vel_prof.v * DR_GEAR;
      if(fabs(mb_vel_prof.w) > 0.001) mb_setpoints.traj_heading += mb_vel_prof.w * DT;
    }
    if (bot_mode == 2) {	// drive square
      if(fabs(mb_vel_prof.v) > 0.001) mb_setpoints.phi_desired += mb_vel_prof.v * DR_TRAJ_GEAR;
      if(fabs(mb_vel_prof.w) > 0.001) mb_setpoints.traj_heading += mb_vel_prof.w * HEADING_GEAR;
      //--old--//
      // if (mb_setpoints.traj_heading > 3.14) {
      //   mb_setpoints.traj_heading -= 2*3.14;
      // }
      // if (mb_setpoints.traj_heading < -3.14) {
      //   mb_setpoints.traj_heading += 2*3.14;
      // }
      //--old--//
      //--New code--//
      // if (mb_setpoints.traj_heading > 6.28) {
      //     mb_setpoints.traj_heading -= 6.28;
      // }
      // if (mb_setpoints.traj_heading < 0) {
      //     mb_setpoints.traj_heading += 6.28;
      // }
      //--New code--//
    }
  }
  return NULL;
}


/*******************************************************************************
* printf_loop()
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
  rc_state_t last_state, new_state; // keep track of last state
  FILE *log_file = fopen("state_log.csv", "w");
  while(rc_get_state()!=EXITING){
    new_state = rc_get_state();
    // check if this is the first time since being paused
    if(new_state==RUNNING && last_state!=RUNNING){
      printf("\nRUNNING: Hold upright to balance.\n");
      printf("                 SENSORS                   |                       ODOMETRY                       |");
      printf("           SETPOINTS            |         CMD          ");
      printf("\n");
      printf("     θ    |");
      printf("     φ    |");
      printf("     γ    |");
      printf("strt_angle|");
      printf("    Ω     |");
      printf("  unwr_γ |");
      printf("   L Enc  |");
      printf("   R Enc  |");
      printf("     X    |");
      printf("     Y    |");
      printf("     ψ    |");
      printf("  φ_SetPt |");
      printf("γ_RC_SetPt|");
      printf("γ_trj_StPt|");
      printf("  fwd_vel |");
      printf("  turn_vel|");
      printf("  Left cmd|");
      printf(" Right cmd|");
      printf("Vel_SetPt |");
      printf("Acc_SetPt |");
      printf("\n");
    }
    else if(new_state==PAUSED && last_state!=PAUSED){
      printf("\nPAUSED\n");
    }
    last_state = new_state;
    if(new_state == RUNNING){
      //Add Print statements here, do not follow with /n
      pthread_mutex_lock(&state_mutex);
      printf("%8.3f  |", mb_state.theta);
      printf("%8.3f  |", mb_state.phi);
      printf("%8.3f  |", mb_state.gamma);
      printf("%8.3f  |", start_angle);
      printf("%8.3f  |", mb_vel_prof.w);
      printf("%8.3f  |", mb_state.unwrapped_gamma);
      printf("%8.3f  |", mb_state.left_encoder);
      printf("%8.3f  |", mb_state.right_encoder);
      printf("%8.3f  |", mb_odometry.x);
      printf("%8.3f  |", mb_odometry.y);
      printf("%8.3f  |", mb_odometry.psi);
      printf("%8.3f  |", mb_setpoints.phi_desired);
      printf("%8.3f  |", mb_setpoints.rc_heading);
      printf("%8.3f  |", mb_setpoints.traj_heading);
      printf("%8.3f  |", mb_setpoints.fwd_velocity);
      printf("%8.3f  |", mb_setpoints.turn_velocity);
      printf("%8.3f  |", mb_state.left_cmd);
      printf("%8.3f  |", mb_state.right_cmd);
      printf("%8.3f  |", mb_vel_prof.v);
      printf("%8.3f  |", mb_vel_prof.acc);
      printf("\r");
      fprintf(log_file,"%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3d,%7.3d\n",
      mb_state.theta, mb_state.phi, mb_odometry.x, mb_odometry.y,
      mb_odometry.psi, mb_state.gamma, mb_odometry.delta_d, mb_setpoints.traj_heading, mb_vel_prof.w,
      mb_state.left_encoder, mb_state.right_encoder);
      pthread_mutex_unlock(&state_mutex);
      fflush(stdout);
    }
    rc_nanosleep(1E9/PRINTF_HZ);
  }
  return NULL;
}

void calc_drag_params(float dist_to_travel, float max_accel) {

}
