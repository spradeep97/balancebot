#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

rc_filter_t D1 = RC_FILTER_INITIALIZER; // Inner loop
rc_filter_t D2 = RC_FILTER_INITIALIZER; // Outer loop
rc_filter_t D3 = RC_FILTER_INITIALIZER; // Steer controller
rc_filter_t D4 = RC_FILTER_INITIALIZER; // Heading controller


/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

double* readMatrixFromFile(char* fileName, int height, int width) {
  FILE* fp = fopen(fileName, "r");
  if (fp == NULL) {
	fprintf(stderr, "Can't open %s.\n", fileName);
	return NULL;
  }
  double val;
  double* M = (double*) malloc(height * width * sizeof(double));
  for(int i = 0; i < height; i++) {
	for(int j = 0; j < width; j++) {
  	if (fscanf(fp, " %lf", &val) != 1) {
    	fprintf(stderr, "Couldn't read value.\n");
    	return NULL;
  	}
  	// Discard the comma without checking.
  	fgetc(fp);
  	M[i * width + j] = val;
	}
  }
  fclose(fp);
  return M;
}

int mb_controller_load_config(){
    /*
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    */
    /* TODO parse your config file here*/
    int width = 5;
    int height = 5;

    double *gain_mat = readMatrixFromFile("pid2.cfg", height, width);

    gains.KPi = gain_mat[0];
    gains.KIi = gain_mat[1];
    gains.KDi = gain_mat[2];
    gains.Tfi = gain_mat[3];
    gains.gi = gain_mat[4];

    gains.KPo = gain_mat[5];
    gains.KIo = gain_mat[6];
    gains.KDo = gain_mat[7];
    gains.Tfo = gain_mat[8];
    gains.go = gain_mat[9];

    gains.KPs = gain_mat[10];
    gains.KIs = gain_mat[11];
    gains.KDs = gain_mat[12];
    gains.Tfs = gain_mat[13];
    gains.gs = gain_mat[14];

    gains.KPh = gain_mat[15];
    gains.KIh = gain_mat[16];
    gains.KDh = gain_mat[17];
    gains.Tfh = gain_mat[18];
    gains.gh = gain_mat[19];

    gains.is_inner   = gain_mat[20];
    gains.is_heading = gain_mat[21];
    gains.bot_mode   = gain_mat[22];

    rc_filter_pid(&D1, gains.KPi, gains.KIi, gains.KDi, gains.Tfi*DT, DT);
    printf("Enabled filter saturation/n");
    rc_filter_enable_saturation(&D1, -1.0, 1.0);
    D1.gain = gains.gi;

    rc_filter_pid(&D2, gains.KPo, gains.KIo, gains.KDo, gains.Tfo*DT, DT);
    rc_filter_enable_saturation(&D2, -0.1, 0.1);
    D2.gain = gains.go;

    rc_filter_pid(&D3, gains.KPs, gains.KIs, gains.KDs, gains.Tfs*DT, DT);
    rc_filter_enable_saturation(&D3, -0.5, 0.5);
    D3.gain = gains.gs;

    rc_filter_pid(&D4, gains.KPh, gains.KIh, gains.KDh, gains.Tfh*DT, DT);
    rc_filter_enable_saturation(&D4, -0.5, 0.5);
    D4.gain = gains.gh;

    free (gain_mat);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
*
*
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t *mb_setpoints, mb_odometry_t* mb_odometry){
    // state measurements
    float theta = mb_state->theta;                  // body angle measured(rad)
    float phi = mb_state->phi;                      // average wheel angle measured(rad)
    /* .......... inner loop control ............ */
    if (gains.is_inner) {
      // body angle set-point and error
      double theta_desired = 0.00;  // 2.9 degrees
      double theta_error = theta_desired - theta;
      // outputs
      double dc_desired = rc_filter_march(&D1, -theta_error);
      mb_state->left_cmd = dc_desired;
      mb_state->right_cmd = dc_desired;
    }
    /* ........... outer and inner loop control ........... */
    if (!gains.is_inner) {
      // wheel angle error
      double phi_error = mb_setpoints->phi_desired - phi;
      // outputs
      double theta_desired = rc_filter_march(&D2, phi_error);
      double theta_error = theta_desired - theta;
      double dc_desired = rc_filter_march(&D1, -theta_error);
      mb_state->left_cmd = dc_desired;
      mb_state->right_cmd = dc_desired;
    }
    /* ...........  Heading control ........... */
    if (gains.is_heading) {
      float heading_correction = mb_heading_controller(mb_setpoints, mb_state);
      mb_state->left_cmd += MOT_1_POL * heading_correction;
      mb_state->right_cmd -= MOT_2_POL * heading_correction;
    }
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
*
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    return 0;
}

float mb_steer_controller(mb_setpoints_t* mb_setpoints, mb_state_t* mb_state) {
  double gamma_error = mb_setpoints->rc_heading - mb_state->enc_gamma;
  float steering_input = rc_filter_march(&D3, gamma_error);
  return steering_input;
}

float mb_heading_controller(mb_setpoints_t* mb_setpoints, mb_state_t* mb_state) {
  float error = (mb_setpoints->traj_heading) - (mb_state->unwrapped_gamma);
  //--New code--//
  // float error = mb_setpoints->traj_heading - mb_state->shifted_gamma;
  // float residue = 6.28 - fabs(error);
  // if (error > -3.14 && error < 0) {
  //     error = abs(error);
  // }
  // if (error < -3.14 && error < 0) {
  //     error = -residue;
  // }
  // if (error > 0 && error < 3.14) {
  //     error = -error;
  // }
  // if (error > 0 && error > 3.14) {
  //     error = residue;
  // }
  //--New code--//
  float heading_correction = rc_filter_march(&D4, error);
  return heading_correction;
}
