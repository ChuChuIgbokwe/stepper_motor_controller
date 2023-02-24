//
// Created by chu-chu on 2/24/23.
//
#include <iostream>

// Define abstract MotorController class
class MotorController {
public:
    virtual ~MotorController() {}
    virtual void step() = 0;
    virtual void set_goal(int goal_pos, int max_vel, int max_acc) = 0;
    virtual bool isSanityCheckFlag() = 0;
};

// Define StepperController class that extends MotorController
class StepperController : public MotorController {
private:
    int initial_pos_;
    int initial_vel_;
    int goal_pos_;
    int max_vel_;
    int max_acc_;
    bool sanity_check_flag_;
public:
    StepperController(int initial_pos, int initial_vel) :
            initial_pos_(initial_pos), initial_vel_(initial_vel), sanity_check_flag_(true) {}
    StepperController(int initial_pos, int initial_vel, int goal_pos, int max_vel, int max_acc) :
            initial_pos_(initial_pos), initial_vel_(initial_vel), goal_pos_(goal_pos), max_vel_(max_vel), max_acc_(max_acc), sanity_check_flag_(true) {}
    void step() override {
        std::cout << "Stepping motor...\n";
        // Implementation details for stepping the motor
    }
    void set_goal(int goal_pos, int max_vel, int max_acc) override {
        goal_pos_ = goal_pos;
        max_vel_ = max_vel;
        max_acc_ = max_acc;
    }
    bool isSanityCheckFlag() override {
        return sanity_check_flag_;
    }
};
