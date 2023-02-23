#include <cassert>
#include <iostream>

#include "../src/StepperController.h"


void test_getCurrentPosition() {
    StepperController sc(0, 0, 100, 10, 1);
    assert(sc.getCurrentPosition() == 0);
}

void test_getCurrentVelocity() {
    StepperController sc(0, 0, 100, 10, 1);
    assert(sc.getCurrentVelocity() == 0);
}

void test_pre_motion_sanity_checks() {
    StepperController sc(0, 0, 100, 10, 1);
    assert(sc.pre_motion_sanity_checks(10.0, -5.0, 0.0, -4.0, 10.0) == false);
    assert(sc.pre_motion_sanity_checks(10.0, -5.0, 0.0, 0.0, 10.0) == false);
    assert(sc.pre_motion_sanity_checks(10.0, -5.0, 0.0, 5.0, 10.0) == false);
    assert(sc.pre_motion_sanity_checks(0.0, 0.0, 100.0, 20.0, 2.0) == true);
    assert(sc.pre_motion_sanity_checks(0.0, 0.0, 100.0, 20.0, 0.0) == false);
    assert(sc.pre_motion_sanity_checks(0.0, 20.0, 100.0, 10.0, 2.0) == false);
    assert(sc.pre_motion_sanity_checks(0.0, -20.0, 50.0, 10.0, 2.0) == true);
    assert(sc.pre_motion_sanity_checks(0.0, -20.0, -50.0, -10.0, 2.0) == false);
    assert(sc.pre_motion_sanity_checks(0.0, 0.0, 100.0, 20.0, -2.0) == false);

}

// Helper function to compare two floats with some precision
bool is_equal(float a, float b, float epsilon = 0.0001) {
    return std::fabs(a - b) < epsilon;
}

void test_step() {
    StepperController sc(0, 0, 100, 10, 1);
    sc.step();
    // Assert that the generated trajectory is not empty
    std::ifstream trajectory_file("../trajectories.csv");
    assert(trajectory_file.peek() != std::ifstream::traits_type::eof());
    trajectory_file.close();
}

void run_all_tests() {
    test_getCurrentPosition();
    test_getCurrentVelocity();
    test_pre_motion_sanity_checks();
    test_step();
}

int main() {
    run_all_tests();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
