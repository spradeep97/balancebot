/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    float delta_sL = (mb_state->wheelAngleL - mb_state->prev_wheelAngleL) * WHEEL_DIAMETER / 2; // in metres
    float delta_sR = (mb_state->wheelAngleR - mb_state->prev_wheelAngleR) * WHEEL_DIAMETER / 2; // in metres
    float delta_psi = (delta_sR - delta_sL) / WHEEL_BASE;
    float delta_d = (delta_sR + delta_sL) / 2;
    mb_odometry->delta_d = delta_d;
    mb_odometry->psi += delta_psi;
    // processing gyro data to be unwrapped and follow the right sign
    //unwrapping gamma
    float shifted = mb_state->gamma;
    if (shifted < 0) {
        shifted = shifted + 2 * M_PI;
    }
    if (fabs(shifted - mb_state->prev_shifted) > 1) {
        if (shifted > 5.0) {
            mb_state->unwrapped_gamma += (2*M_PI - shifted) + (mb_state->prev_shifted - 0);
        }
        if (shifted < 0.7) {
            mb_state->unwrapped_gamma += shifted;
        }
    } else {
        mb_state->unwrapped_gamma += (shifted - (mb_state->prev_shifted));
    }
    //--unwrapping gamma
    // float state_gamma_copy = -mb_state->gamma;
    // if (state_gamma_copy > 0) {
    //     state_gamma_copy = state_gamma_copy - 2 * M_PI;
    // }
    // float delta_gamma = state_gamma_copy - mb_state->prev_gamma;
    // if (fabs(delta_gamma - delta_psi) > THRESH) {
    //   mb_odometry->fused_theta = state_gamma_copy;
    // } else {
    //   mb_odometry->fused_theta = mb_odometry->psi;
    // }
    //
    // float delta_fused_theta = mb_odometry->fused_theta - mb_odometry->prev_fused_theta;
    // float delta_x = delta_d * cos(mb_odometry->fused_theta + delta_fused_theta / 2);
    // float delta_y = delta_d * sin(mb_odometry->fused_theta + delta_fused_theta / 2);
    float delta_x = delta_d * cos(mb_odometry->psi + delta_psi / 2);
    float delta_y = delta_d * sin(mb_odometry->psi + delta_psi / 2);
    mb_odometry->x += delta_x;
    mb_odometry->y += delta_y;
    // mb_odometry->prev_fused_theta = mb_odometry->fused_theta;
    // mb_state->prev_gamma = state_gamma_copy;
    mb_state->prev_shifted = shifted;
}

float mb_clamp_radians(float angle){
    return 0;
}
