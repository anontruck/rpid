/**
 * @file main.c
 * @author Zach Smith (zachary.smith@sjsu.edu)
 * @brief Test for rpid.c module. Done for CMPE 242 Spring 2019.
 * @version 0.1
 * @date 2019-05-06
 * 
 * @copyright Copyright (c) 2019
 * 
 * @todo actually do test
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "rpid.h"

static struct rpid pid;

int main(int argc, char **argv)
{
    //if (argc < 2)
    //{
        //printf("No file given");
        //return EXIT_FAILURE;
    //}

    //FILE* file = fopen(argv[1], "r");

    rpid_init(&pid, 1.0, -100.0, 100.0);
    rpid_set_gains(&pid, 1.0, 2.0, 4.0);
    rpid_update_setpoint(&pid, 10.0);
    return 0;
}