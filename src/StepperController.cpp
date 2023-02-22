//
// Created by chu-chu on 2/21/23.
//

#include "StepperController.h"

StepperController::StepperController(float initial_position, float initial_velocity) :
_initial_position(initial_position),
_initial_velocity(initial_velocity),
_current_position(initial_position),
_current_velocity(initial_velocity){}

void StepperController::set_goal(float position, float velocity, float acceleration) {
    _goal_position = position;
    _max_velocity = velocity;
    _max_acceleration = acceleration;
}


float StepperController::getCurrentPosition() const{
    return _current_position;
}

float StepperController::getCurrentVelocity() const {
    return _current_velocity;
}


bool StepperController::pre_motion_sanity_checks(float initial_position, float initial_velocity, float goal_position,
                                                     float max_velocity, float max_acceleration) {
        if (initial_position < 0.0 || initial_position > 100.0) {
            std::cerr << "Initial position out of range!" << std::endl;
            return false;
        }
        if (initial_velocity < 0.0 || initial_velocity > max_velocity) {
            std::cerr << "Initial velocity out of range!" << std::endl;
            return false;
        }
        if (goal_position < 0.0 || goal_position > 100.0) {
            std::cerr << "Goal position out of range!" << std::endl;
            return false;
        }
        if (max_velocity < 0.0 || max_velocity > 10.0) {
            std::cerr << "Max velocity out of range!" << std::endl;
            return false;
        }
        if (max_acceleration < 0.0 || max_acceleration > 10.0) {
            std::cerr << "Max acceleration out of range!" << std::endl;
            return false;
        }
        return true;
    }



void StepperController::_set_direction() {
    if (_goal_position < 0){
        _direction = -1.0;
    }
    else{
        _direction = 1.0;
    }
}

void StepperController::_calculate_distances() {
    _total_distance = std::fabs(_goal_position - _initial_position);
    _acceleration_distance = _initial_velocity * _acceleration_time + 0.5 * _max_acceleration * std::pow(_acceleration_time, 2);
    _deceleration_distance = 0.5 * _max_acceleration * std::pow(_deceleration_time, 2);
    _cruising_distance = _total_distance - _acceleration_distance - _deceleration_distance;
    _cruising_distance = std::fmax(0.0, _cruising_distance);
}

void StepperController::_calculate_times() {
    _acceleration_time = (_max_velocity - _initial_velocity) / _max_acceleration;
    _deceleration_time = _max_velocity / _max_acceleration;
    _cruising_time = _cruising_distance / _max_velocity;
    _total_time = _acceleration_time + _cruising_time + _deceleration_time;
}


float StepperController::calculate_acceleration_time(float max_velocity, float initial_velocity,
                                                            float max_acceleration) {
    return (max_velocity - initial_velocity) / max_acceleration;
}

float StepperController::calculate_acceleration_distance(float initial_velocity, float
acceleration_time, float max_acceleration){
    return initial_velocity * acceleration_time + 0.5 * max_acceleration * std::pow
            (acceleration_time, 2);

}

float StepperController::calculate_deceleration_time(float max_velocity, float max_acceleration) {
    return max_velocity / max_acceleration;
}

float StepperController::calculate_deceleration_distance(float max_acceleration, float deceleration_time) {
    return 0.5 * max_acceleration * std::pow(deceleration_time, 2);
}



void StepperController::step() {
    _set_direction();
    bool triangular_profile_flag = false;
    _current_position = _initial_position;
    float total_distance = std::fabs(_goal_position - _initial_position);
    float remaining_distance = _goal_position - _current_position;

    // Acceleration time and distance
    float acceleration_time = (_max_velocity - _initial_velocity) / _max_acceleration;
//    _acceleration_time = calculate_acceleration_time(_max_velocity, _initial_velocity, _max_acceleration);
    float acceleration_distance = _initial_velocity * acceleration_time + 0.5 * _max_acceleration * std::pow(acceleration_time, 2);
//    _acceleration_distance = calculate_acceleration_distance(_initial_velocity, _acceleration_time, _max_acceleration);

    // Deceleration time and distance
    float deceleration_time = _max_velocity / _max_acceleration;
    float deceleration_distance = 0.5 * _max_acceleration * std::pow(deceleration_time, 2);

    // Cruising time and distance
    float cruising_distance = total_distance - _acceleration_distance - deceleration_distance;
    cruising_distance = std::fmax(0.0, cruising_distance);
    float cruising_time = cruising_distance / _max_velocity;

    float total_time = _acceleration_time + cruising_time + deceleration_time;
    std::ofstream trajectory_file;
    trajectory_file.open("../trajectories.csv");
    if (!trajectory_file.is_open()) {
        std::cerr << "Failed to open file for writing!" << std::endl;
        return;
    }
    _current_velocity = _initial_velocity;
    _current_acceleration = _max_acceleration;
    const float time_step = 0.1;
    float time_elapsed = 0.0;

    while (time_elapsed <= total_time) {
        if (time_elapsed < _acceleration_time) {
            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", "
                            << _current_acceleration << std::endl;

            // Acceleration
            _current_acceleration = _max_acceleration;
            // Velocity
            _current_velocity += _direction * _max_acceleration * time_step;
            _current_velocity = std::fmin(_max_velocity, _current_velocity);
            // Position
            _current_position += _current_velocity * time_step + 0.5 * _current_acceleration * std::pow(time_step, 2);
            //Update time elapsed
            time_elapsed += time_step;

        }
        else if (time_elapsed < _acceleration_time + cruising_time) {
            // Acceleration
            _current_acceleration = 0;
            // Velocity
            _current_velocity = _direction * _max_velocity;
            // Position
            _current_position += (_current_velocity * time_step);

            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", " <<
                            _current_acceleration << std::endl;
            //Update time elapsed
            time_elapsed += time_step;

        }
        else {
            std::cout << "\ndecelerating " << time_elapsed << std::endl;
            _current_acceleration = -_max_acceleration;
            _current_velocity += _current_acceleration * time_step;
            _current_position += _direction * _current_velocity * time_step + 0.5 * _current_acceleration * std::pow
                    (time_step, 2);

            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", "
                            << _current_acceleration << std::endl;
            time_elapsed += time_step;

        }
    }
    trajectory_file.close();
}

void StepperController::step() {
    float direction;

    if (_goal_position < 0){
        direction = -1.0;
    }
    else{
        direction = 1.0;
    }

    // Set current position, velocity and acceleration to their initial values
    _current_position = _initial_position;
    _current_velocity = _initial_velocity;
    _current_acceleration = _max_acceleration;


    float total_distance = std::fabs(_goal_position - _initial_position);
    float remaining_distance = _goal_position - _current_position;

    float acceleration_time = (_max_velocity - _initial_velocity) / _max_acceleration;
    float acceleration_distance = _initial_velocity * acceleration_time + 0.5 * _max_acceleration * std::pow
            (acceleration_time, 2);

    float deceleration_time = _max_velocity / _max_acceleration;
    float deceleration_distance = 0.5 * _max_acceleration * std::pow(deceleration_time, 2);

    float cruising_distance = total_distance - acceleration_distance - deceleration_distance;
    cruising_distance = std::fmax(0, cruising_distance);
    float cruising_time = cruising_distance / _max_velocity;


    float total_time = acceleration_time + cruising_time + deceleration_time;


    std::cout << "acceleration_time = " << acceleration_time << std::endl;
    std::cout << "acceleration_distance = " << acceleration_distance << std::endl;
    std::cout << "deceleration_time = " << deceleration_time << std::endl;
    std::cout << "deceleration_distance = " << deceleration_distance << std::endl;
    std::cout << "total_time = " << total_time << std::endl;
    std::cout << "total_distance = " << total_distance << std::endl;
    std::cout << "cruise_time = " << cruising_time << std::endl;
    std::cout << "cruising_distance = " << cruising_distance << std::endl;
    std::cout << "remaining_distance = " << remaining_distance << std::endl;


    std::ofstream trajectory_file("../trajectories.csv");
    if (!trajectory_file) {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }

    float time_step = 0.1;
    float time_elapsed = 0.0;
    float distance_covered = total_distance - remaining_distance;

//    while (remaining_distance >= 1e-1) {
    while (time_elapsed <= total_time) {
//        if (current_position < acceleration_distance) {
        if (time_elapsed < acceleration_time) {
            std::cout << "\naccelerating " << time_elapsed << std::endl;
            std::cout << "accelerating current_position = " << _current_position << std::endl;
            std::cout << "accelerating current_velocity = " << _current_velocity << std::endl;
            std::cout << "remaining_distance = " << remaining_distance << std::endl;
            std::cout << "distance_covered = " << distance_covered << std::endl;
            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", "
                            << _current_acceleration << std::endl;
            // velocity
            _current_velocity += direction * _max_acceleration * time_step;
            _current_velocity = std::min(_max_velocity, _current_velocity);

            // Acceleration
            _current_acceleration = _max_acceleration;

            // position
            _current_position += _current_velocity * time_step + 0.5 * _current_acceleration * std::pow(time_step, 2);

            // compute remaining distance
            remaining_distance = _goal_position - _current_position;
            distance_covered = total_distance - remaining_distance;


            time_elapsed += time_step;

        }
//        else if (current_position < acceleration_distance + cruising_distance) {
        else if (time_elapsed < acceleration_time + cruising_time) {
            std::cout << "\ncruising " << time_elapsed << std::endl;

            _current_velocity = direction * _max_velocity;
            _current_acceleration = 0;
            _current_position += (_current_velocity * time_step);


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
            _current_position += direction * _current_velocity * time_step + 0.5 * _current_acceleration * std::pow
                    (time_step, 2);


            remaining_distance = _goal_position - _current_position;
            distance_covered = total_distance - remaining_distance;


            trajectory_file << time_elapsed << ", " << _current_position << ", " << _current_velocity << ", "
                            << _current_acceleration << std::endl;
            time_elapsed += time_step;

            std::cout << "decelerating current_position = " << _current_position << std::endl;
            std::cout << "decelerating current_velocity = " << _current_velocity << std::endl;
            std::cout << "decelerating current_acceleration = " << _current_acceleration << std::endl;
            std::cout << "decelerating remaining_distance = " << remaining_distance << std::endl;
            std::cout << "distance_covered = " << distance_covered << std::endl;


        }
    }
    trajectory_file.close();
}

