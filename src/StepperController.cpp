//
// Created by chu-chu on 2/21/23.
//

#include "StepperController.h"

/// @brief Constructor for StepperController class
///@param initial_position The initial position of the stepper motor
///@param initial_velocity The initial velocity of the stepper motor
///@param goal_position The target position of the stepper motor
///@param max_velocity The maximum velocity that the stepper motor can reach
///@param max_acceleration The maximum acceleration that the stepper motor can achieve
/// @details Initializes StepperController object with the given initial position, initial velocity, goal position,
///maximum velocity, and maximum acceleration. Also creates a socket for real-time communication and
///graph plotting if the _graph_real_time_flag and _communication_flag are both set to true.
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
        _trapezoid_curve_debug_flag(false),
        _graph_real_time_flag(true),
        _communication_flag(true){

    //

    //Only create a socket if both flags are true
    if(isCommunicationFlag() and isGraphRealTimeFlag()){
        std::cout<<"Creating Socket"<<std::endl;
        struct sockaddr_in serv_addr;
        // Create a socket
        if ((_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            perror("Failed to create socket");
            exit(EXIT_FAILURE);
        }

        // Configure the server address
        memset(&serv_addr, '0', sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(8082);

        // Connect to the server
        if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
            perror("Invalid address/ Address not supported");
            exit(EXIT_FAILURE);
        }

        if (connect(_socket_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
            perror("Connection failed");
            exit(EXIT_FAILURE);

        }
    }
}


StepperController::StepperController(float initial_position, float initial_velocity) :
        _initial_position(initial_position),
        _initial_velocity(initial_velocity),
        _current_position(initial_position),
        _current_velocity(initial_velocity),
        _distance_and_time_debug_flag(false),
        _trapezoid_curve_debug_flag(false),
        _graph_real_time_flag(false),
        _communication_flag(false){}

StepperController::~StepperController() {

}

/// @brief Setter method for the goal position, maximum velocity and maximum acceleration of the stepper motor
/// @param position the desired goal position of the stepper motor
/// @param velocity the maximum velocity of the stepper motor
/// @param acceleration the maximum acceleration of the stepper motor
void StepperController::set_goal(float position, float velocity, float acceleration) {
    _goal_position = position;
    _max_velocity = velocity;
    _max_acceleration = acceleration;
    _current_acceleration = acceleration;
}

/// @brief Getter method for the current position of the stepper motor
/// @return the current position of the stepper motor
float StepperController::getCurrentPosition() const {
    return _current_position;
}


/// @brief Getter method for the current velocity of the stepper motor
///@return the current velocity of the stepper motor
float StepperController::getCurrentVelocity() const {
    return _current_velocity;
}

/// @brief This function generates a trajectory for a stepper motor controller to move from its initial position
/// the goal position with given maximum velocity and acceleration. The function performs pre-motion sanity checks
///to ensure that the motion is feasible. If the motion is not feasible, the function returns without generating a
/// trajectory.
/// @details The function calculates the total distance to be covered, the remaining distance to the goal position,
///the acceleration time and distance, deceleration time and distance, cruising distance and time, and total time for
///the motion. It then generates the trajectory by computing the position and velocity of the motor at each time step.
///The generated trajectory is written to a file named "trajectories.csv" in the "data" directory.
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

    std::ofstream trajectory_file("../data/trajectories.csv");
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
/// @param acceleration_time The time it takes to accelerate to the maximum velocity.
///@param acceleration_distance The distance traveled during the acceleration phase.
///@param deceleration_time The time it takes to decelerate to a stop.
///@param deceleration_distance The distance traveled during the deceleration phase.
///@param total_time The total time of the motion.
///@param total_distance The total distance traveled during the motion.
///@param cruising_time The time spent cruising at maximum velocity.
///@param cruising_distance The distance traveled during the cruising phase.
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

/// @brief Performs various sanity checks before starting a motion and returns true if they all pass, false otherwise
/// @param initial_position the initial position of the stepper motor
/// @param initial_velocity the initial velocity of the stepper motor
/// @param goal_position the desired goal position of the stepper motor
/// @param max_velocity the maximum velocity of the stepper motor
/// @param max_acceleration the maximum acceleration of the stepper motor
/// @return true if all checks pass, false otherwise return false
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
/// @param time_elapsed the time elapsed since the beginning of the deceleration phase
/// @param remaining_distance the distance remaining until the stepper motor reaches the goal position
/// @param distance_covered the distance covered by the stepper motor so far
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
/// @param time_elapsed the time elapsed since the beginning of the deceleration phase
/// @param remaining_distance the distance remaining until the stepper motor reaches the goal position
/// @param distance_covered the distance covered by the stepper motor so far
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
/// @param time_elapsed the time elapsed since the beginning of the deceleration phase
/// @param remaining_distance the distance remaining until the stepper motor reaches the goal position
/// @param distance_covered the distance covered by the stepper motor so far
void StepperController::print_deceleration_debug_values(float &time_elapsed, float &remaining_distance,
                                                        float &distance_covered) {
    std::cout << "\ndecelerating " << time_elapsed << std::endl;
    std::cout << "decelerating current_position = " << _current_position << std::endl;
    std::cout << "decelerating current_velocity = " << _current_velocity << std::endl;
    std::cout << "decelerating current_acceleration = " << _current_acceleration << std::endl;
    std::cout << "decelerating remaining_distance = " << remaining_distance << std::endl;
    std::cout << "distance_covered = " << distance_covered << std::endl;

}

///@brief Getter function for _distance_and_time_debug_flag member variable.
///@return bool Returns the current value of _distance_and_time_debug_flag.
bool StepperController::isDistanceAndTimeDebugFlag() const {
    return _distance_and_time_debug_flag;
}

///@brief Setter function for _distance_and_time_debug_flag member variable.
///@param distanceAndTimeDebugFlag The boolean value to set
void StepperController::setDistanceAndTimeDebugFlag(bool distanceAndTimeDebugFlag) {
    _distance_and_time_debug_flag = distanceAndTimeDebugFlag;
}



/// @brief Accelerates the stepper motor given the current velocity, position and acceleration, and the maximum
/// acceleration
/// @param current_velocity the current velocity of the stepper motor
/// @param current_position the current position of the stepper motor
/// @param current_acceleration the current acceleration of the stepper motor
/// @param max_acceleration the maximum deceleration that can be applied to the motor
/// @param time_step the amount of time that has elapsed since the last
/// @details The current acceleration is set to the max
/// acceleration, the formula for current velocity is the first equation of motion V = U + a*t. or current velocity =
/// initial velocity + acceleration * time. Since the current_velocity is the initial velocity originally, we simply
/// add a*t at each time step. Position is the second equation of motion S = u*t + 0.5 * a*t^2. Like current velocity
/// we increase our position at each time step.
void StepperController::accelerate(float &current_velocity, float &max_velocity, float &current_position, float &
current_acceleration, float max_acceleration, float time_step) {
    current_acceleration = max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_velocity = std::min(max_velocity, current_velocity);

    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}


/// @brief This function is responsible for updating the position, velocity and acceleration of the motor during the cruise phase.
/// @param current_velocity the current velocity of the stepper motor
/// @param current_position the current position of the stepper motor
/// @param current_acceleration the current acceleration of the stepper motor
/// @param max_acceleration the maximum deceleration that can be applied to the motor
/// @param time_step the amount of time that has elapsed since the last
/// @details The current acceleration is set to zero since
/// the motor is moving at it's max velocity. The current velocity is set to the maximum velocity and the current
/// position the second equation of motion. However, a is 0 so S = u*t + 0.5*a*t^2 becomes S = u*t. We update the
/// position at every time step
void StepperController::cruise(float &current_velocity, float &current_position, float max_velocity, float &
current_acceleration, float time_step) {
    current_acceleration = 0;
    current_velocity = max_velocity;
    current_position += current_velocity * time_step;
}



/// @brief Decelerates the stepper motor given the current velocity, position and acceleration, and the maximum acceleration
/// @param current_velocity the current velocity of the stepper motor
/// @param current_position the current position of the stepper motor
/// @param current_acceleration the current acceleration of the stepper motor
/// @param max_acceleration the maximum deceleration that can be applied to the motor
/// @param time_step the amount of time that has elapsed since the last update
/// @details The current acceleration is set to the
/// negative of max acceleration since the motor is slowing down, the formula for current velocity is the first equation
/// of motion V = U + a*t. or current velocity = initial velocity + acceleration * time. Since the current_velocity is
/// the initial velocity originally, we simply add a*t at each time step. Position is the second equation of motion
/// S = u*t + 0.5 * a*t^2. Like current velocity we increase our position at each time step.
void StepperController::decelerate(float &current_velocity, float &current_position, float &current_acceleration,
                                   float max_acceleration, float time_step) {
    current_acceleration = -max_acceleration;
    current_velocity += current_acceleration * time_step;
    current_position += current_velocity * time_step + 0.5 * current_acceleration * std::pow(time_step, 2);
}

/// @brief Updates the trajectory by appending current time, position, velocity and acceleration to the provided output file
/// @param trajectory_file the output file stream to write to
/// @param time_elapsed the time elapsed since the beginning of the motion
/// @param current_position the current position of the stepper motor
/// @param current_velocity the current velocity of the stepper motor
/// @param current_acceleration the current acceleration of the stepper motor

void StepperController::update_trajectory(std::ofstream &trajectory_file, float &time_elapsed,
                                          float &current_position, float &current_velocity,
                                          float &current_acceleration) {
    trajectory_file << time_elapsed << ", " << current_position << ", " << current_velocity << ", "
                    << current_acceleration << std::endl;
}

/// @brief This method computes and writes the time steps, positions, velocities and accelerations of the stepper motor
/// to a csv file. The while loop runs while the time elapsed is less than the total time to get to the stepper motor's
/// goal position. If the _trapezoid_curve_debug_flag is toggled to true, debug messages are printed to screen
/// @param total_time the total time taken to complete the trajectory.
/// @param time_elapsed the time elapsed since the start of the trajectory.
/// @param acceleration_time the time taken to accelerate the stepper motor.
/// @param cruising_time the time taken to cruise the stepper motor.
/// @param time_step the time step at which the stepper motor is updated.
/// @param trajectory_file the file to write the updated trajectory.
/// @param remaining_distance the distance remaining for the stepper motor to reach the goal position.
/// @param distance_covered the total distance covered by the stepper motor.
/// @param total_distance the total distance to be covered by the stepper motor.
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

        }
        else if (time_elapsed < acceleration_time + cruising_time) {
            if (_trapezoid_curve_debug_flag) {
                print_cruising_debug_values(time_elapsed, remaining_distance, distance_covered);
            }

            update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                              _current_acceleration);
            cruise(_current_velocity, _current_position, _max_velocity, _current_acceleration, time_step);


            }
        else {
            if (_trapezoid_curve_debug_flag) {
                print_deceleration_debug_values(time_elapsed, remaining_distance, distance_covered);
            }
            update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                              _current_acceleration);
            decelerate(_current_velocity, _current_position, _current_acceleration, _max_acceleration, time_step);

            }
        //Only send data over socket if both flags are true
        if (isCommunicationFlag() and isGraphRealTimeFlag()){
            send_data(time_elapsed);
        }


        // compute remaining distance
        remaining_distance = _goal_position - _current_position;
        distance_covered = total_distance - remaining_distance;
        time_elapsed += time_step;
    }
    update_trajectory(trajectory_file, time_elapsed, _current_position, _current_velocity,
                      _current_acceleration);
}

/// @brief Getter method for _sanity_check_flag.
/// @return The current value of _sanity_check_flag.
bool StepperController::isSanityCheckFlag() const {
    return _sanity_check_flag;
}

/// @brief Setter method for _sanity_check_flag.
/// @param sanityCheckFlag the value to be set for _sanity_check_flag.
void StepperController::setSanityCheckFlag(bool sanityCheckFlag) {
    _sanity_check_flag = sanityCheckFlag;
}

/// @brief Calculates the total distance between the initial and goal positions.
/// @return The total distance as a float value.
float StepperController::calculate_total_distance() {
    return std::fabs(_goal_position - _initial_position);
}

/// @brief Calculates the time it takes to accelerate from the initial velocity to the maximum velocity.
/// @return The acceleration time as a float value.
float StepperController::calculate_acceleration_time() {
    return std::fabs((_max_velocity - _initial_velocity) / _max_acceleration);
}

/// @brief Calculates the distance traveled during acceleration given the acceleration time.
/// @param acceleration_time The acceleration time as a float value.
/// @return The acceleration distance as a float value.
float StepperController::calculate_acceleration_distance(float acceleration_time) {
    return _initial_velocity * acceleration_time + 0.5 * _max_acceleration * std::pow(acceleration_time, 2);
}

/// @brief Calculates the time it takes to decelerate from the maximum velocity to 0 velocity.
/// @return The deceleration time as a float value.
float StepperController::calculate_deceleration_time() {
    return std::fabs(_max_velocity / _max_acceleration);
}

/// @brief Calculates the distance traveled during deceleration given the deceleration time.
/// @param deceleration_time The deceleration time as a float value.
/// @return The deceleration distance as a float value.
float StepperController::calculate_deceleration_distance(float deceleration_time) {
    return 0.5 * _max_acceleration * std::pow(deceleration_time, 2);
}

/// @brief Calculates the distance that will be traveled while cruising at maximum velocity.
/// @param total_distance The total distance to be traveled as a float value.
/// @param acceleration_distance The distance traveled during acceleration as a float value.
/// @param deceleration_distance The distance traveled during deceleration as a float value.
/// @return The cruising distance as a float value.
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

/// @brief Calculates the time spent cruising at maximum velocity.
/// @param cruising_distance The cruising distance as a float value.
/// @return The cruising time as a float value.
float StepperController::calculate_cruising_time(float cruising_distance){
    return cruising_distance / _max_velocity;
}

/// @brief Calculates the remaining distance to the goal position.
/// @return The remaining distance as a float value.
float StepperController::calculate_remaining_distance(){
        return _goal_position - _current_position;
}

/// @brief Calculates the total time it takes to complete the motion given the acceleration, cruising, and deceleration times.
/// @param acceleration_time  The acceleration time as a float value.
/// @param cruising_time  The cruising time as a float value.
/// @param deceleration_time The deceleration time as a float value.
/// @return
float StepperController::calculate_total_time(float acceleration_time, float cruising_time, float deceleration_time){
    if (acceleration_time + cruising_time + deceleration_time <= 0) {
        std::cerr << "WARNING: Cannot follow trapezoidal velocity curve with the given parameters. total_time is less"
                     " than 0"
                  << std::endl;
    }
    return acceleration_time + cruising_time + deceleration_time;
}

/// @brief Sends data to the socket with the given time elapsed, current position, velocity, and acceleration.
/// @param time_elapsed The time elapsed since the start of the motion
void StepperController::send_data(float time_elapsed) {
    size_t buffer_size = 1024;
    char* buffer = (char*) malloc(buffer_size);
    if (buffer == NULL) {
        perror("Failed to allocate memory");
        exit(EXIT_FAILURE);
    }
    memset(buffer, 0, buffer_size);
    int n = snprintf(buffer, buffer_size, "%.3f,%.3f,%.3f,%.3f\n", time_elapsed, _current_position, _current_velocity, _current_acceleration);
    printf("Sending data: %s", buffer);
    if (send(_socket_fd, buffer, n, 0) < 0) {
        perror("Failed to send data");
        exit(EXIT_FAILURE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    printf("Data sent successfully.\n");
    free(buffer);
}

/// @brief Returns the value of the _graph_real_time_flag attribute.
/// @return The value of _graph_real_time_flag.
bool StepperController::isGraphRealTimeFlag() const {
    return _graph_real_time_flag;
}

/// @brief Sets the value of the _graph_real_time_flag attribute to the given value.
/// @param graphRealTimeFlag The new value for the _graph_real_time_flag attribute.
void StepperController::setGraphRealTimeFlag(bool graphRealTimeFlag) {
    _graph_real_time_flag = graphRealTimeFlag;
}

/// @brief Returns the value of the _communication_flag attribute.
/// @return The value of _communication_flag.
bool StepperController::isCommunicationFlag() const {
    return _communication_flag;
}

/// @brief Sets the value of the _communication_flag attribute to the given value.
/// @param communicationFlag The new value for the _communication_flag attribute.
void StepperController::setCommunicationFlag(bool communicationFlag) {
    _communication_flag = communicationFlag;
}


