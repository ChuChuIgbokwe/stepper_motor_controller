//
// Created by chu-chu on 2/21/23.
//

#ifndef URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
#define URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
#include <iostream>
#include <math.h>
#include <fstream>

class StepperController {
public:
    //Constructor
//    StepperController();
//    StepperController(int initial_position, int initial_velocity);
    StepperController(float initial_position, float initial_velocity);

    float getCurrentPosition() const;

    float getCurrentVelocity() const;

//    void set_goal(int position, int velocity, int acceleration);
    void set_goal(float position, float velocity, float acceleration);

    void step();

    bool pre_motion_sanity_checks(float initial_position, float initial_velocity, float
    goal_position, float max_velocity, float max_acceleration);

private:
    float _initial_position;
    float _initial_velocity;
    float _current_position;
    float _current_velocity;
    float _current_acceleration;
    float _goal_position;
    float _max_velocity;
    float _max_acceleration;
    float _acceleration_time;
    float _acceleration_distance;
    float _deceleration_time;
    float _deceleration_distance;
    float _total_time;
    float _total_distance;
    float _cruising_time;
    float _cruising_distance;
    float _remaining_distance;
    float _direction;


    void print_acceleration_debug_values();

    void _set_direction();

    void _calculate_distances();

    void _calculate_times();

    float calculate_acceleration_time(float max_velocity, float initial_velocity, float max_acceleration);

    float calculate_acceleration_distance(float initial_velocity, float acceleration_time, float max_acceleration);

    float calculate_deceleration_time(float max_velocity, float max_acceleration);

    float calculate_deceleration_distance(float max_acceleration, float deceleration_time);
};



#endif //URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
