//
// Created by chu-chu on 2/21/23.
//

#include "StepperController.h"


StepperController::StepperController(float initial_position, float initial_velocity, float goal_position, float
max_velocity, float max_acceleration) :
        _initial_position(initial_position),
        _initial_velocity(initial_velocity),
        _goal_position(goal_position),
        _max_velocity(max_velocity),
        _max_acceleration(max_acceleration),
        _current_position(initial_position),
        _current_velocity(initial_velocity),
        _distance_and_time_debug_flag(false),
        _trapezoid_curve_debug_flag(false) {}



StepperController::StepperController(float initial_position, float initial_velocity) :
        _initial_position(initial_position),
        _initial_velocity(initial_velocity),
        _current_position(initial_position),
        _current_velocity(initial_velocity),
        _distance_and_time_debug_flag(false),
        _trapezoid_curve_debug_flag(false) {}

StepperController::~StepperController() {

}

void StepperController::set_goal(float position, float velocity, float acceleration) {
    _goal_position = position;
    _max_velocity = velocity;
    _max_acceleration = acceleration;
    _current_acceleration = acceleration;

}
/// @brief a getter method to return the current position
/// @return
float StepperController::getCurrentPosition() const {
    return _current_position;
}


/// @brief a getter method to return the current velocity
/// @return
float StepperController::getCurrentVelocity() const {
    return _current_velocity;
}


void StepperController::step() {
    _sanity_check_flag = pre_motion_sanity_checks(_initial_position, _initial_velocity,
                                                  _goal_position, _max_velocity,
                                                  _max_acceleration);
    if (!_sanity_check_flag) {
        return;
    }

    float total_distance = calculate_total_distance();
    float remaining_distance = calculate_remaining_distance();

    float acceleration_time = calculate_acceleration_time();
    float acceleration_distance = calculate_acceleration_distance(acceleration_time);

    float deceleration_time = calculate_deceleration_time();
    float deceleration_distance = calculate_deceleration_distance(deceleration_time);

    float cruising_distance = calculate_cruising_distance(total_distance, acceleration_distance, deceleration_distance);
    float cruising_time = calculate_cruising_time(cruising_distance);

    // Compute total time and check if it's feasible
    float total_time = calculate_total_time(acceleration_time, cruising_time, deceleration_time);

    float distance_covered = total_distance - remaining_distance;

    if (_distance_and_time_debug_flag) {
        debug_print_motion_parameters(acceleration_time, acceleration_distance,
                                      deceleration_time, deceleration_distance, total_time, total_distance,
                                      cruising_time,
                                      cruising_distance);
    }

    std::ofstream trajectory_file("../trajectories.csv");
    if (!trajectory_file) {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }

    float time_step = 0.1;
    float time_elapsed = 0.0;
    _current_acceleration = _max_acceleration;


    generate_trajectory(total_time, time_elapsed, acceleration_time, cruising_time, time_step, trajectory_file,
                        remaining_distance, distance_covered, total_distance);
    trajectory_file.close();
}


/// @brief This method prints out the various motion parameters to make debugging easier, it is
/// toggled on and off with _distance_and_time_debug_flag
/// @param acceleration_time
/// @param acceleration_distance
/// @param deceleration_time
/// @param deceleration_distance
/// @param total_time
/// @param total_distance
/// @param cruising_time
/// @param cruising_distance
void StepperController::debug_print_motion_parameters(float acceleration_time, float acceleration_distance,
                                                      float deceleration_time, float deceleration_distance,
                                                      float total_time, float total_distance, float cruising_time,
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

/// @brief This method if the initial parameters are viable for the stepper motor to have a trapezoidal velocity curve.
/// @param initial_position
/// @param initial_velocity
/// @param goal_position
/// @param max_velocity
/// @param max_acceleration
/// @return
bool StepperController::pre_motion_sanity_checks(float initial_position, float initial_velocity, float goal_position,
                                                 float max_velocity, float max_acceleration) {
    if (max_velocity == 0.0) {
        std::cerr << "Error, max velocity is 0. Motor will cruise forever" << std::endl;
        return false;
    }
    if (max_velocity > 0 and initial_velocity > 0 and max_velocity < initial_velocity) {
        std::cerr << "Error, max velocity cannot be less than initial velocity" << std::endl;
        return false;
    }
    if (max_velocity > 0 and initial_velocity < 0 and goal_position < initial_position) {
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
    if (goal_position < 0 or max_velocity < 0 or max_acceleration < 0) {
        std::cerr << "Error, this class doesn't handle negative goal positions, max velocities or max accelerations"
                  << std::endl;
        return false;
    }
    return true;
}


/// @brief This method prints out the various acceleration values to make debugging the acceleration stage easier, it is
///// toggled on and off with _trapezoid_curve_debug_flag
/// @param time_elapsed
/// @param remaining_distance
/// @param distance_covered
void StepperController::print_acceleration_debug_values(float &time_elapsed, float &remaining_distance, float &
distance_covered) {
    std::cout << "\naccelerating " << time_elapsed << std::endl;
    std::cout << "accelerating current_position = " << _current_position << std::endl;
    std::cout << "accelerating current_velocity = " << _current_velocity << std::endl;
    std::cout << "accelerating current_acceleration = " << _current_acceleration << std::endl;
    std::cout << "remaining_distance = " << remaining_distance << std::endl;
    std::cout << "distance_covered = " << distance_covered << std::endl;

}


/// @brief This method prints out the various cruising values to make debugging the cruising stage easier, it is
/// toggled on and off with _trapezoid_curve_debug_flag
/// @param time_elapsed
/// @param remaining_distance
/// @param distance_covered
void StepperController::print_cruising_debug_values(float &time_elapsed, float &remaining_distance,
                                                    float &distance_covered) {
    std::cout << "\ncruising " << time_elapsed << std::endl;
    std::cout << "cruising current_position = " << _current_position << std::endl;
    std::cout << "cruising current_velocity = " << _current_velocity << std::endl;
    std::cout << "cruising current_acceleration = " << _current_acceleration << std::endl;
    std::cout << "cruising remaining_distance = " << remaining_distance << std::endl;
    std::cout << "distance_covered = " << distance_covered << std::endl;
}

/// @brief This method prints out the various deceleration values to make debugging the deceleration stage easier, it is
/// toggled on and off with _trapezoid_curve_debug_flag
/// @param time_elapsed
/// @param remaining_distance
/// @param distance_covered
void StepperController::print_deceleration_debug_values(float &time_elapsed, float &remaining_distance,
                                                        float &distance_covered) {
    std::cout << "\ndecelerating " << time_elapsed << std::endl;
    std::cout << "decelerating current_position = " << _current_position << std::endl;
    std::cout << "decelerating current_velocity = " << _current_velocity << std::endl;
    std::cout << "decelerating current_acceleration = " << _current_acceleration << std::endl;
    std::cout << "decelerating remaining_distance = " << remaining_distance << std::endl;
    std::cout << "distance_covered = " << distance_covered << std::endl;

}

bool StepperController::isDistanceAndTimeDebugFlag() const {
    return _distance_and_time_debug_flag;
}

void StepperController::setDistanceAndTimeDebugFlag(bool distanceAndTimeDebugFlag) {
    _distance_and_time_debug_flag = distanceAndTimeDebugFlag;
}


/// @brief This method handles the calculations for the acceleration phase. current acceleration is set to the max
/// acceleration, the formula for current velocity is the first equation of motion V = U + a*t. or current velocity =
/// initial velocity + acceleration * time. Since the current_velocity is the initial velocity originally, we simply
/// add a*t at each time step. Position is the second equation of motion S = u*t + 0.5 * a*t^2. Like current velocity
/// we increase our position at each time step.
/// @param current_velocity
/// @param max_velocity
/// @param current_position
/// @param current_acceleration
/// @param max_acceleration
/// @param time_step
void StepperController::accelerate(float &current_velocity, float &max_velocity, float &current_position, float &
current_acceleration, float max_acceleration, float time_step) {
    current_acceleration = max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_velocity = std::min(max_velocity, current_velocity);

    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}


/// @brief This method handles the calculations for the cruising phase. current acceleration is set to zero since
/// the motor is moving at it's max velocity. The current velocity is set to the maximum velocity and the current
/// position the second equation of motion. However, a is 0 so S = u*t + 0.5*a*t^2 becomes S = u*t. We update the
/// position at every time step
/// @param current_velocity
/// @param current_position
/// @param max_velocity
/// @param current_acceleration
/// @param time_step
void StepperController::cruise(float &current_velocity, float &current_position, float max_velocity, float &
current_acceleration, float time_step) {
    current_acceleration = 0;
    current_velocity = max_velocity;
    current_position += current_velocity * time_step;
}



/// @brief This method handles the calculations for the deceleration phase. current acceleration is set to the
// negative of max acceleration since the motor is slowing down, the formula for current velocity is the first equation
/// of motion V = U + a*t. or current velocity = initial velocity + acceleration * time. Since the current_velocity is
/// the initial velocity originally, we simply add a*t at each time step. Position is the second equation of motion
/// S = u*t + 0.5 * a*t^2. Like current velocity we increase our position at each time step.
/// @param current_velocity
/// @param current_position
/// @param current_acceleration
/// @param max_acceleration
/// @param time_step
void StepperController::decelerate(float &current_velocity, float &current_position, float &current_acceleration,
                                   float max_acceleration, float time_step) {
    current_acceleration = -max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}

/// @brief This method encapsulates writing the time elapsed, positions, velocities and accelerations to an already
/// open csv file
/// @param trajectory_file
/// @param time_elapsed
/// @param current_position
/// @param current_velocity
/// @param current_acceleration
void StepperController::update_trajectory(std::ofstream &trajectory_file, float &time_elapsed,
                                          float &current_position, float &current_velocity,
                                          float &current_acceleration) {
    trajectory_file << time_elapsed << ", " << current_position << ", " << current_velocity << ", "
                    << current_acceleration << std::endl;
}

/// @brief This method computes and writes the time steps, positions, velocities and accelerations of the stepper motor
/// to a csv file. The while loop runs while the time elapsed is less than the total time to get to the stepper motor's
/// goal position. If the _trapezoid_curve_debug_flag is toggled to true, debug messages are printed to screen
/// @param total_time
/// @param time_elapsed
/// @param acceleration_time
/// @param cruising_time
/// @param time_step
/// @param trajectory_file
/// @param remaining_distance
/// @param distance_covered
/// @param total_distance
void StepperController::generate_trajectory(float &total_time, float &time_elapsed, float &acceleration_time, float &
cruising_time, float &time_step, std::ofstream &trajectory_file, float &
remaining_distance, float &distance_covered, float &total_distance) {
    while (time_elapsed <= total_time) {
        if (time_elapsed < acceleration_time) {
            if (_trapezoid_curve_debug_flag) {
                print_acceleration_debug_values(time_elapsed, remaining_distance, distance_covered);
            }
            update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                              _current_acceleration);
            accelerate(_current_velocity, _max_velocity, _current_position, _current_acceleration, _max_acceleration,
                       time_step);

        } else if (time_elapsed < acceleration_time + cruising_time) {
            if (_trapezoid_curve_debug_flag) {
                print_cruising_debug_values(time_elapsed, remaining_distance, distance_covered);
            }

            update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                              _current_acceleration);
            cruise(_current_velocity, _current_position, _max_velocity, _current_acceleration, time_step);


        } else {
            if (_trapezoid_curve_debug_flag) {
                print_deceleration_debug_values(time_elapsed, remaining_distance, distance_covered);
            }
            update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                              _current_acceleration);
            decelerate(_current_velocity, _current_position, _current_acceleration, _max_acceleration, time_step);

        }
        // compute remaining distance
        remaining_distance = _goal_position - _current_position;
        distance_covered = total_distance - remaining_distance;
        time_elapsed += time_step;
    }
    update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                      _current_acceleration);
}

bool StepperController::isSanityCheckFlag() const {
    return _sanity_check_flag;
}

void StepperController::setSanityCheckFlag(bool sanityCheckFlag) {
    _sanity_check_flag = sanityCheckFlag;
}


float StepperController::calculate_total_distance() {
    return std::fabs(_goal_position - _initial_position);
}


float StepperController::calculate_acceleration_time() {
    return std::fabs((_max_velocity - _initial_velocity) / _max_acceleration);
}

float StepperController::calculate_acceleration_distance(float acceleration_time) {
    return _initial_velocity * acceleration_time + 0.5 * _max_acceleration * std::pow(acceleration_time, 2);
}


float StepperController::calculate_deceleration_time() {
    return std::fabs(_max_velocity / _max_acceleration);
}

float StepperController::calculate_deceleration_distance(float deceleration_time) {
    return 0.5 * _max_acceleration * std::pow(deceleration_time, 2);
}

float StepperController::calculate_cruising_distance(float total_distance, float acceleration_distance,
                                                     float deceleration_distance) {
    float cruising_distance = total_distance - acceleration_distance - deceleration_distance;
    if (cruising_distance == 0.0) {
        std::cout << "WARNING: Cannot follow trapezoidal velocity curve with the given parameters. Cruising distance "
                     "is 0"
                  << std::endl;
    }
    return std::fmax(0, cruising_distance);
}

float StepperController::calculate_cruising_time(float cruising_distance){
    return cruising_distance / _max_velocity;
}


float StepperController::calculate_remaining_distance(){
        return _goal_position - _current_position;
}


// Compute total time and check if it's feasible
float StepperController::calculate_total_time(float acceleration_time, float cruising_time, float deceleration_time){
    if (acceleration_time + cruising_time + deceleration_time <= 0) {
        std::cerr << "WARNING: Cannot follow trapezoidal velocity curve with the given parameters. total_time is less"
                     " than 0"
                  << std::endl;
    }
    return acceleration_time + cruising_time + deceleration_time;
}


