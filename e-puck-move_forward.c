#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>

/* Motor devices */
static WbDeviceTag left_motor, right_motor;

/* E-puck angular speed in rad/s */
#define MAX_SPEED 6.28
#define TANGENSIAL_SPEED 0.12874

int time_step;

/* function to get simulator time step */
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

/* function to set motor velocity to move forward */
void motorStop() {
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void motorMoveForward() {
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/* function to move forward by distance in meters */
void moveForward(double distance) {
    double duration = distance / TANGENSIAL_SPEED;
    printf("Moving forward for %.2f meters\n", distance);

    motorMoveForward();

    double start_time = wb_robot_get_time();
    do {
        wb_robot_step(time_step);
    } while (wb_robot_get_time() < start_time + duration);

    motorStop();
}

/* function to initialize robot stuff */
static void init_robot() {
    time_step = get_time_step();

    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
}

/* main function */
int main(int argc, char **argv) {
    /* necessary to initialize webots stuff */
    wb_robot_init();

    init_robot();

    /* Move forward 4 meters */
    moveForward(2.0);

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return EXIT_SUCCESS;
}
