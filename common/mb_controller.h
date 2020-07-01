#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#include <rc/math.h>
#define CFG_PATH "pid2.cfg"

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry);
int mb_controller_cleanup();
float mb_steer_controller(mb_setpoints_t* mb_setpoints, mb_state_t* mb_state);
float mb_heading_controller(mb_setpoints_t* mb_setpoints, mb_state_t* mb_state);

#endif
