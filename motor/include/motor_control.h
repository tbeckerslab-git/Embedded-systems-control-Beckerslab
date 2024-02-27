#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#endif

#include <dynamixel_sdk.h>
#include <iostream> // user input library
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // Used with usleep

// Function prototypes can be declared here.
int initializeDynamixel();
void setGoalPosition(int position);
int readCurrentPosition();

#endif // MOTOR_CONTROL_H
