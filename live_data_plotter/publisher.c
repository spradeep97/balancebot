#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include <lcm/lcm.h>
#include "raw_mpu_t.h"
#define READ_HZ 100

rc_mpu_data_t data;

//Callback function
void print_data(void){
		printf("%6.2f %6.2f %6.2f", data.accel[0], data.accel[1], data.accel[2]);
		printf("%6.1f %6.1f %6.1f \n", data.gyro[0], data.gyro[1], data.gyro[2]);
		printf("%6.1f %6.1f %6.1f |", data.dmp_TaitBryan[TB_PITCH_X],
			   	 data.dmp_TaitBryan[TB_ROLL_Y],data.dmp_TaitBryan[TB_YAW_Z]);
}

int main(){
    lcm_t *lcm = lcm_create(NULL);
    raw_mpu_t raw_reading = {0};
		rc_mpu_config_t conf = rc_mpu_default_config();
		rc_mpu_initialize(&data, conf);
		rc_mpu_initialize_dmp(&data, conf);
		rc_mpu_set_dmp_callback(&print_data); //Set Callback function

	  rc_set_state(RUNNING);
	  while(rc_get_state() != EXITING){
			rc_mpu_read_accel(&data);
			rc_mpu_read_gyro(&data);
      raw_reading.accel[0] = data.accel[0];
      raw_mpu_t_publish(lcm, "RAW_MPU", &raw_reading);
      // printf("%6.2f\r", raw_reading.accel[0]);
			fflush(stdout);
		  rc_nanosleep(1E9/READ_HZ); //OR
	  }
    lcm_destroy(lcm);
		return 0;
}
