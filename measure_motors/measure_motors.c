/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Option A: Capture encoder readings, current readings, timestamps etc.
*       to a file to analyze and determine motor parameters
*
*       Option B: Capture the same information within get_motor_params and follow
*       on its structure for obtaining the parameters and printing them in your
*       terminal.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

#define Vmax 12
#define CPR 48


FILE* f1;

int get_motor_params(int motor, int polarity, float resistance, float dtime_s);

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main() {
    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0) < -2) return -1;

    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler() == -1) {
        fprintf(stderr, "ERROR: failed to start signal handler\n");
        return -1;
    }

    // if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
    //     fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
    //     return -1;
    // }

    //initialaize Motors
    if (mb_motor_init() < 0) {
        fprintf(stderr, "ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    // initialize enocders
    if(rc_encoder_eqep_init() == -1) {
        fprintf(stderr, "ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init() == -1) {
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    rc_set_state(RUNNING);

    /**********************************************************************
    [OPTION A] TODO : Loop and Save data to a file and migrate that info
                      to python or matlab

    while(rc_get_state()!=EXITING){
        rc_nanosleep(1E9);
        //get data
        //save to file
    }
    // close file
    **********************************************************************/

    /**********************************************************************
    [OPTION B] TODO : Follow on the guide within get_motor_params and
                      construct it accordingly. Then run it for each motor
                      given you know its resistance.
    **********************************************************************/

    int pass_mot1, pass_mot2;
    float dtime_s = 5; // 5sec is usuall enough but you can change
    double resistance1 = 5.2;
    double resistance2 = 5.2;
    pass_mot1 = get_motor_params(1, ENC_1_POL, resistance1, dtime_s);
    pass_mot2 = get_motor_params(2, ENC_2_POL, resistance2, dtime_s);

    // exit cleanly
    mb_motor_disable();
    mb_motor_cleanup();
    rc_adc_cleanup();
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file(); // remove pid file LAST
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// [OPTION B] TODO : Fill in get_motor_params to obtain motor parameters

int get_motor_params(int motor, int polarity, float resistance, float dtime_s) {
    // Parameters to feed in:
    /***************************************************************************************************************
    > motor : Motor number referring to right or left motor
    > polarity : Encoder polarity, you can remove this as an argument and just use the global variable from the defs
    > resistance : The resistance value you measured off the motors as we will use this in the calculations
    > dtime_s : The time to complete the transient measurements
    ***************************************************************************************************************/

    // Pass flag you can manipulate to return success or fail of motor measurement
    int pass_flag = -1;

    // Variable defs;

    float duty = 0.99; //Running at full duty cycle
    float encoder_ticks, speed, noload_speed, mot_constant, stall_torque, shaft_fric, shaft_inertia;
    double noload_current;
    double dt, start_time, time_elapse, prevtime, time_const;
    int got_time_const = 0;

    // First run for steady state data and obtain all info attainable from that.
    rc_encoder_write(motor, 0);
    mb_motor_set(motor, duty);
    rc_nanosleep((int)dtime_s * 1E9);
    encoder_ticks = polarity * rc_encoder_eqep_read(motor);
    noload_speed = encoder_ticks * (2.0 * 3.14) / (GEAR_RATIO * ENCODER_RES) / dtime_s;
    noload_current = mb_motor_read_current(motor - 1); // pin = motor - 1

    // Things you would be able to calculate from the three recorded values aboveu
    double V = Vmax * duty;
    mot_constant = (V - noload_current * resistance) / noload_speed;
    stall_torque = mot_constant * V / resistance;
    shaft_fric = (mot_constant * V / noload_speed - mot_constant * mot_constant) / resistance; // b

    // Turn off the motor after steady state calcs are done
    mb_motor_set(motor, 0.0);
    rc_nanosleep(2E9);

    // TODO: Transient State measurements and calculations
    rc_encoder_write(motor, 0); // Reset the encoder
    mb_motor_set(motor, duty); // Set the motor again at max dc

    // We need to time the transient run now in order to obtain the time constant.
    // We will keep monitoring our spin speed in loops and once it exceeds or matches 63%
    // of our no load speed calculated above, we record the time as the time constant.

    start_time = (double)(rc_nanos_since_epoch()) * 1.0E-9;
    time_elapse = 0.0; // Current loop time
    prevtime = 0.0; // Prev loop time, needed to get dt

    // Our while loop termination condition is the max run time dtime_s we provide as an argument to the function
    while(time_elapse < dtime_s) {
        time_elapse = (double)(rc_nanos_since_epoch()) * 1.0E-9 - start_time;
        dt = time_elapse - prevtime;
        prevtime = time_elapse;
        encoder_ticks = rc_encoder_read(motor);
        speed = encoder_ticks / dt;
        if(!got_time_const && speed > (0.63 * noload_speed)){
            printf("Got time constant\n");
            got_time_const = 1;
            time_const = time_elapse;
        }
        rc_nanosleep(1E7);
    }
    mb_motor_disable();
    shaft_inertia = time_const * shaft_fric; // calculated using transfer function of 28 in Lecture 2 Handout

    // Finally, print all the info you obtained
    printf("[ No Load Speed (rpm) : %3.4f, No Load Current (A) : %3.4lf,  Stall Torque (N.m) : %3.4f ]\n", noload_speed, noload_current, stall_torque);
    printf("[ Motor Constant K : %3.4f, Shaft Friction : %3.4f,  Shaft Inertia (Kg.m^2) : %1.4e ]\n\n", mot_constant, shaft_fric, shaft_inertia);

    pass_flag = 1;

    return pass_flag;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
