cmake_minimum_required(VERSION 3.5)
project(stepper_controller_tests)

set(CMAKE_CXX_STANDARD 11)

# Add the source files to the executable
add_executable(stepper_controller_tests test_stepper_controller.cpp ../src/StepperController.cpp
        ../src/StepperController.h)

# Find and link against the required libraries
#find_package(Boost REQUIRED COMPONENTS program_options)
#target_link_libraries(MyProgram PRIVATE Boost::program_options)
