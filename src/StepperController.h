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
    StepperController(float initial_position, float initial_velocity);

    StepperController(float initial_position, float initial_velocity, float goal_position, float max_velocity,
                      float max_acceleration);

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
    bool _sanity_check_flag;
public:
    bool isSanityCheckFlag() const;

    void setSanityCheckFlag(bool sanityCheckFlag);

private:


    void print_acceleration_debug_values();

    void _set_direction();

    void _calculate_distances();

    void _calculate_times();

    float calculate_acceleration_time(float max_velocity, float initial_velocity, float max_acceleration);

    float calculate_acceleration_distance(float initial_velocity, float acceleration_time, float max_acceleration);

    float calculate_deceleration_time(float max_velocity, float max_acceleration);

    float calculate_deceleration_distance(float max_acceleration, float deceleration_time);

    bool _distance_and_time_debug_flag;
    bool _trapezoid_curve_debug_flag;

    void debug_print_motion_parameters(float acceleration_time, float acceleration_distance, float deceleration_time,
                                       float deceleration_distance, float total_time, float total_distance,
                                       float cruising_time, float cruising_distance) const;

    void print_acceleration_debug_values(float &time_elapsed, float &remaining_distance, float &distance_covered);

    void print_cruising_debug_values(float &time_elapsed, float &remaining_distance, float &distance_covered);

    void print_deceleration_debug_values(float &time_elapsed, float &remaining_distance, float &distance_covered);

    bool isDistanceAndTimeDebugFlag() const;

    void setDistanceAndTimeDebugFlag(bool distanceAndTimeDebugFlag);

    void accelerate(float &current_velocity, float &max_velocity, float &current_position, float &current_acceleration,
                    float max_acceleration, float time_step);

    void cruise(float &current_velocity, float &current_position, float max_velocity, float &current_acceleration,
                float time_step);

    void
    decelerate(float &current_velocity, float &current_position, float &current_acceleration, float max_acceleration,
               float time_step);

    void
    update_trajectory(std::ofstream &trajectory_file, float &time_elapsed, float &current_position,
                      float &current_velocity,
                      float &current_acceleration);

    void generate_trajectory(float &total_time, float &time_elapsed, float &acceleration_time, float &cruising_time,
                             float &time_step, std::ofstream &trajectory_file, float &remaining_distance,
                             float &distance_covered, float &total_distance);
};


#endif //URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
