//
// Created by chu-chu on 2/21/23.
//

#include "StepperController.h"

StepperController::StepperController(float initial_position, float initial_velocity) :
_initial_position(initial_position),
_initial_velocity(initial_velocity),
_current_position(initial_position),
_current_velocity(initial_velocity),
_distance_and_time_debug_flag(false){}

void StepperController::set_goal(float position, float velocity, float acceleration) {
    _goal_position = position;
    _max_velocity = velocity;
    _max_acceleration = acceleration;
    _current_acceleration = acceleration;

}


float StepperController::getCurrentPosition() const{
    return _current_position;
}

float StepperController::getCurrentVelocity() const {
    return _current_velocity;
}

void StepperController::debug_print_motion_parameters(float acceleration_time, float acceleration_distance,
                                                      float deceleration_time, float deceleration_distance, float total_time, float total_distance, float cruising_time,
                                                      float cruising_distance) const {
    std::cout << "acceleration_time = " << acceleration_time << std::endl;
    std::cout << "acceleration_distance = " << acceleration_distance << std::endl;
    std::cout << "deceleration_time = " << deceleration_time << std::endl;
    std::cout << "deceleration_distance = " << deceleration_distance << std::endl;
    std::cout << "total_time = " << total_time << std::endl;
    std::cout << "total_distance = " << total_distance << std::endl;
    std::cout << "cruise_time = " << cruising_time << std::endl;
    std::cout << "cruising_distance = " << cruising_distance << std::endl;
}


bool StepperController::pre_motion_sanity_checks(float initial_position, float initial_velocity, float goal_position,
                                                 float max_velocity, float max_acceleration) {
    if (max_velocity == 0.0) {
        std::cerr << "Error, max velocity is 0. Motor will cruise forever" << std::endl;
        return false;
    }if (max_velocity > 0 and initial_velocity > 0 and max_velocity < initial_velocity) {
        std::cerr << "Error, max velocity cannot be less than initial velocity" << std::endl;
        return false;
    }
    if (max_velocity > 0 and initial_velocity < 0 and goal_position < initial_position ) {
        std::cerr << "Error, it's not possible to hit a goal position less than the initial position with a "
                     "positive max velocity" << std::endl;
        return false;
    }
    if (max_velocity < 0 and initial_velocity < 0 and std::fabs(max_velocity) < std::fabs(initial_velocity)) {
        std::cerr << "Error, both initial and goal velocities are negative but the max velocity is less than the "
                     "initial velocity since we're moving in the other direction" <<
        std::endl;
        return false;
    }
    if (max_acceleration < 0.0) {
        std::cerr << "Max acceleration is cannot be negative." << std::endl;
        return false;
    }

    if (max_acceleration == 0 or max_velocity == 0) {
        std::cerr << "Potential Division by Zero Error, max_acceleration or max_velocity is zero." << std::endl;
        return false;
    }
    if (initial_position == goal_position) {
        std::cerr << "Error, the motor won't move, initial_position and goal_position are the same"
                  << std::endl;
        return false;
    }
    if (goal_position < 0 or  max_velocity < 0 or max_acceleration < 0 ) {
        std::cerr << "Error, this class doesn't handle negative goal positions, max velocities or max accelerations"
                  << std::endl;
        return false;
    }
    return true;
}


void StepperController::step() {

    bool sanity_check_flag;
    sanity_check_flag = pre_motion_sanity_checks(_initial_position, _initial_velocity, _goal_position, _max_velocity,
                                                 _max_acceleration);
    if(!sanity_check_flag){
        return;
    }

    float total_distance = std::fabs(_goal_position - _initial_position);
    float remaining_distance = _goal_position - _current_position;

    float acceleration_time = std::fabs((_max_velocity - _initial_velocity) / _max_acceleration);
    float acceleration_distance = _initial_velocity * acceleration_time + 0.5 * _max_acceleration * std::pow(acceleration_time, 2);


    float deceleration_time = std::fabs(_max_velocity / _max_acceleration);
    float deceleration_distance = std::fabs(0.5 * _max_acceleration * std::pow(deceleration_time, 2));

    float cruising_distance = total_distance - acceleration_distance - deceleration_distance;
    cruising_distance = std::fmax(0, cruising_distance);
    float cruising_time = cruising_distance / _max_velocity;

    // Compute total time and check if it's feasible
    float total_time = acceleration_time + cruising_time + deceleration_time;
    if (cruising_time == 0) {
        std::cout << "WARNING: Cannot follow trapezoidal velocity curve with the given parameters. Cruising time is 0"
        << std::endl;
    }
    if (total_time <= 0) {
        std::cerr << "WARNING: Cannot follow trapezoidal velocity curve with the given parameters. total_time is less"
                     " than 0"
                  << std::endl;
        return;
    }

    if (_distance_and_time_debug_flag){
        debug_print_motion_parameters(acceleration_time, acceleration_distance,
                deceleration_time, deceleration_distance, total_time, total_distance, cruising_time,
                cruising_distance);
    }

    std::ofstream trajectory_file("../trajectories.csv");
    if (!trajectory_file) {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }

    float time_step = 0.1;
    float time_elapsed = 0.0;
    float distance_covered = total_distance - remaining_distance;

    while (time_elapsed <= total_time) {
        if (time_elapsed < acceleration_time) {
            std::cout << "\naccelerating " << time_elapsed << std::endl;
            std::cout << "accelerating current_position = " << _current_position << std::endl;
            std::cout << "accelerating current_velocity = " << _current_velocity << std::endl;
            std::cout << "remaining_distance = " << remaining_distance << std::endl;
            std::cout << "distance_covered = " << distance_covered << std::endl;
//            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", " << _current_acceleration << std::endl;

            // Acceleration
//            _current_acceleration = _max_acceleration;

            // velocity
//            _current_velocity += _current_acceleration * time_step;
//            _current_velocity = std::min(_max_velocity, _current_velocity);

            // position
//            _current_position += _current_velocity * time_step + 0.5 * _current_acceleration * std::pow(time_step, 2);


            update_trajectory(trajectory_file, time_elapsed,
                    _current_position, _current_velocity, _current_acceleration);
            accelerate(_current_velocity, _current_position, _current_acceleration,
                       _max_acceleration, time_step);
            // compute remaining distance
            remaining_distance = _goal_position - _current_position;
            distance_covered = total_distance - remaining_distance;

            time_elapsed += time_step;

        }
        else if (time_elapsed < acceleration_time + cruising_time) {
            std::cout << "\ncruising " << time_elapsed << std::endl;

            _current_acceleration = 0;
            _current_velocity = _max_velocity;
            _current_position += _current_velocity * time_step;

            remaining_distance = _goal_position - _current_position;
            distance_covered = total_distance - remaining_distance;

            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", " <<
                                                                                                              _current_acceleration << std::endl;

            time_elapsed += time_step;

            std::cout << "cruising current_position = " << _current_position << std::endl;
            std::cout << "cruising current_velocity = " << _current_velocity << std::endl;
            std::cout << "cruising current_acceleration = " << _current_acceleration << std::endl;
            std::cout << "cruising remaining_distance = " << remaining_distance << std::endl;
            std::cout << "distance_covered = " << distance_covered << std::endl;
        }
        else {
            std::cout << "\ndecelerating " << time_elapsed << std::endl;
            _current_acceleration = -_max_acceleration;
            _current_velocity += _current_acceleration * time_step;
            _current_position += _current_velocity * time_step + 0.5 * _current_acceleration * std::pow(time_step, 2);

            remaining_distance = _goal_position - _current_position;
            distance_covered = total_distance - remaining_distance;

            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", "
                            << _current_acceleration << std::endl;
            time_elapsed += time_step;

            std::cout << "decelerating current_position = " << _current_position << std::endl;
            std::cout << "decelerating current_velocity = " << _current_velocity << std::endl;
//            std::cout << "decelerating current_acceleration = " << _current_acceleration << std::endl;
            std::cout << "decelerating remaining_distance = " << remaining_distance << std::endl;
            std::cout << "distance_covered = " << distance_covered << std::endl;


        }
    }
    trajectory_file.close();
}

bool StepperController::isDistanceAndTimeDebugFlag() const {
    return _distance_and_time_debug_flag;
}

void StepperController::setDistanceAndTimeDebugFlag(bool distanceAndTimeDebugFlag) {
    _distance_and_time_debug_flag = distanceAndTimeDebugFlag;
}

void StepperController::accelerate(float& current_velocity, float& current_position, float& current_acceleration,
                float max_acceleration, float time_step) {
    current_acceleration = max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}

void StepperController::cruise(float& current_velocity, float& current_position, float max_velocity, float time_step) {
    current_velocity = max_velocity;
    current_position += current_velocity * time_step;
}

void StepperController::decelerate(float& current_velocity, float& current_position, float& current_acceleration,
                float max_acceleration, float time_step) {
    current_acceleration = -max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}

void StepperController::update_trajectory(std::ofstream& trajectory_file, float time_elapsed,
                       float current_position, float current_velocity, float current_acceleration) {
    trajectory_file << time_elapsed << ", " << current_position << ", " << current_velocity << ", "
                    << current_acceleration << std::endl;
}

void generate_trajectory(float total_time, float acceleration_time, float cruising_time,
                         float max_velocity, float max_acceleration, float goal_position,
                         float time_step, std::ofstream& trajectory_file) {
    float remaining_distance = goal_position;
    float distance_covered = 0.0;
    float current_position = 0.0;
    float current_velocity = 0.0;
    float current_acceleration = 0.0;
    float time_elapsed = 0.0;
    float total_distance = goal_position;

    while (time_elapsed <= total_time) {
        if (time_elapsed < acceleration_time) {
//            update_trajectory(trajectory_file, time_elapsed, current_position, current_velocity, current_acceleration);
//            accelerate(current_velocity, current_position, current_acceleration, max_acceleration, time_step);

        }
        else if (time_elapsed < acceleration_time + cruising_time) {
            std::cout << "\ncruising " << time_elapsed << std::endl;
            update_trajectory(trajectory_file, time_elapsed, current_position, current_velocity, current_acceleration);
            cruise(current_velocity, current_position, max_velocity, time_step);

        }
        else {
            std::cout << "\ndecelerating " << time_elapsed << std::endl;
//            update_trajectory(trajectory_file, time_elapsed, current_position, current_velocity, current_acceleration);
            decelerate(current_velocity, current_position, current_acceleration, max_acceleration, time_step);

        }
        remaining_distance = goal_position - current_position;
        distance_covered = total_distance - remaining_distance;
        time_elapsed += time_step;
    }
}
