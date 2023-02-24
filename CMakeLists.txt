cmake_minimum_required(VERSION 3.24)
project(urban_machine_generic_stepper_motor_controller)

set(CMAKE_CXX_STANDARD 23)

add_executable(urban_machine_generic_stepper_motor_controller  main.cpp src/StepperController.cpp src/StepperController.h src/MotorController.cpp)

