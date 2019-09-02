#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "main.h"
#include "Gimbal_Task.h"

extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_relax_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_init_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_stop_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_gyro_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_key_to_align_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);
static void gimbal_encoder_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set);

#endif
