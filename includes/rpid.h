/**
 * @file rpid.h
 * @author Zach Smith (zachary.smith@sjsu.edu)
 * @brief Simple implementation of a P.I.D controller for CMPE 242 Spring 2019.
 * @version 0.1
 * @date 2019-05-06
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef RPID_H
#define RPID_H

#include <inttypes.h>
#include <stdbool.h>

//#define KERN_LEN 3

//struct diff_kernel {
    //uint32_t next_index;
    //double buffer[KERN_LEN]; // track past data
    //double kernel[KERN_LEN]; // central difference kernel
//};

#define IBUFF_SIZE 2

struct rpid {
    double kp;
    double ki;
    double kd;
    double min;
    double max;
    double setpoint;
    double t_sample;
    double error_sum;
    uint32_t buff_index;
    double input_buff[IBUFF_SIZE];
    //struct diff_kernel dk;
};

int rpid_init(struct rpid *pid, double sample_time, double min, double max);
int rpid_set_gains(struct rpid *pid, double p, double i, double d);
int rpid_update_setpoint(struct rpid *pid, double setpoint);
double rpid_update(struct rpid *pid, double fb);

#endif /* RPID_H */