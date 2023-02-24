cmake_minimum_required(VERSION 3.21)
project(sim_motor)

set(CMAKE_CXX_STANDARD 23)

add_executable(sim_motor  main.cpp src/StepperController.cpp src/StepperController.h src/MotorController.cpp)

