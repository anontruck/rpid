/**
 * @file rpid.c
 * @author Zach Smith (zachary.smith@sjsu.edu)
 * @brief Simple implementation of a P.I.D controller for CMPE 242 Spring 2019.
 * @version 0.1
 * @date 2019-05-06
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "rpid.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

//static const double PI = 3.14159265358979323846;

/* forward function declarations */
//static double diff(struct diff_kernel *kern, double next_in);
static double central_diff(double x0, double x2, double coeff);

/**
 * @brief Initializes the P.I.D data structure by assigning sample rate, 
 * saturation limits, and initializing the derivative kernel.
 * 
 * @param pid pointer to P.I.D object
 * @param sample_time period in which the controller is periodically updated
 * @param min controller output saturation maximum
 * @param max controller output saturation minimum
 * @return int -1 on error, 1 otherwise
 */
int rpid_init(struct rpid *pid, double sample_time, double min, double max)
{
    if ((pid == NULL)       ||
        (sample_time <= 0)  ||
        (min <= 0.0)        ||
        (max <= min)) {
        return -1;
    }
    
    // initialize everything to 0
    memset((void *)pid, 0.0, sizeof(struct rpid));

    // set some defaults
    pid->kp = 1.0;
    pid->min = min;
    pid->max = max;
    pid->setpoint = 0.0;
    pid->t_sample = sample_time;
    pid->buff_index = 0;

    return 1;
}

/**
 * @brief 
 * 
 * @param pid The P.I.D object to update
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Derivative gain
 * @return int -1 on invalid entry
 */
int rpid_set_gains(struct rpid *pid, double p, double i, double d)
{
    if ((pid == NULL) ||
        (p < 0.0) ||
        (i < 0.0) ||
        (d < 0.0)) {
        return -1;
    }

    // re-scale the error summation so we don't get a jolt on our output if Ki
    // changes significantly
    if ((pid->ki != 0.0) && (i != 0.0))
        pid->error_sum *= pid->ki / i;
    
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    return 1;
}

/**
 * @brief Update the given P.I.D controllers setpoint
 * 
 * @param pid 
 * @param setpoint 
 * @return int 
 */
int rpid_update_setpoint(struct rpid *pid, double setpoint)
{
    if (pid == NULL) return-1;
    pid->setpoint = setpoint;
    return 1;
}

/**
 * @brief 
 * 
 * @param pid 
 * @param fb The system feedback value, NOT the error term
 * @return double The controllers next output
 */
double rpid_update(struct rpid *pid, double fb)
{
    if (pid == NULL) return 0.0;

    double fb_dt = 0.0;
    double error = 0.0;

    // extract the oldest input value (n-2) for taking the derivative
    uint32_t index = pid->buff_index;
    double u0 = pid->input_buff[index];
    // then push new value
    pid->input_buff[index] = fb;
    // and lastly update the index
    pid->buff_index = (index + 1) % IBUFF_SIZE;

    // calculate how much each gain contributes
    error = pid->setpoint - fb;

    //fb_dt = diff(&pid->kernel, fb);

    // take the derivative using the feedback signal instead of the error to
    // avoid derivative kick
    fb_dt = central_diff(u0, fb, pid->t_sample);
    pid->error_sum += pid->t_sample * error;

    return (pid->kp * error) + (pid->ki * pid->error_sum) - (pid->kd * fb_dt);
}

/**
 * @brief Computes the derivative of signal 'u' using the central difference
 * theorem. Note that since this looks at 'future' inputs, the value calculated
 * is actually that of the previous sample. Tested in Matlab.
 * 
 * @remark While use of the central difference theorem is implied, according to
 * Stanley T. Birchfield's textbook on computer vision this is equivalent to a
 * gaussian derivative kernel when the kernel size is 3. This is independent of
 * the given variance because VAR=0.5 is the only variance that gives this 
 * kernel size.
 * 
 * @param u0 
 * @param u2 
 * @param delta_x 
 * @return double 
 */
static double central_diff(double u0, double u2, double delta_x)
{
    double coeff = 0.0;

    if (delta_x == 0.0)
        delta_x = 1.0;

    coeff = (1.0 / (2.0 * delta_x));
    
    return coeff * (u2 - u0);
}

///**
 //* @brief 
 //* 
 //* @param kern 
 //* @param next_in 
 //* @return double 
 //*/
//static double diff(struct diff_kernel *kern, double fb)
//{
    //double result = 0.0;

    //if (kern == NULL) return 0.0;

    //return result;
//}