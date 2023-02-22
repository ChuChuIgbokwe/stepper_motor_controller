//
// Created by chu-chu on 2/21/23.
//

#ifndef URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
#define URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H


class StepperController {
public:
    //Constructor
    StepperController();
    StepperController(int initial_position, int initial_velocity);

    float getCurrentPosition();

    float getCurrentVelocity();

    void set_goal(int position, int velocity, int acceleration);

    void step(){}

    bool pre_motion_sanity_checks(float initial_position, float initial_velocity, float
    goal_position, float max_velocity, float max_acceleration);

    void print_values();

private:
    int _initial_position;
    int _initial_velocity;
    int _current_position;
    int _current_velocity;
    int _current_acceleration;
    int _goal_position;
    int _max_velocity;
    int _max_acceleration;

};

};


#endif //URBAN_MACHINE_GENERIC_STEPPER_MOTOR_CONTROLLER_STEPPERCONTROLLER_H
