#include <iostream>
#include "src/StepperController.h"
#include <boost/program_options.hpp>
#include <cstdlib>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {

//StepperController stepperController(150, -50);
//stepperController.set_goal(850,75,10);
//stepperController.step();
//    StepperController stepperController(-150, -50, 550, 75, 10);
//    StepperController stepperController(-150, -50);
//    stepperController.set_goal(550, 75, 10);
//    stepperController.step();


    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("initial-pos", po::value<int>(), "initial position of stepper motor")
            ("initial-vel", po::value<int>(), "initial velocity of stepper motor")
            ("goal-pos", po::value<int>(), "goal position of stepper motor")
            ("max-vel", po::value<int>(), "maximum velocity of stepper motor")
            ("max-acc", po::value<int>(), "maximum acceleration of stepper motor");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    int initial_pos = 0;
    int initial_vel = 0;
    int goal_pos = 0;
    int max_vel = 0;
    int max_acc = 0;

    if (vm.count("initial-pos")) {
        initial_pos = vm["initial-pos"].as<int>();
    }

    if (vm.count("initial-vel")) {
        initial_vel = vm["initial-vel"].as<int>();
    }

    if (vm.count("goal-pos")) {
        goal_pos = vm["goal-pos"].as<int>();
    }

    if (vm.count("max-vel")) {
        max_vel = vm["max-vel"].as<int>();
    }

    if (vm.count("max-acc")) {
        max_acc = vm["max-acc"].as<int>();
    }

    StepperController controller(initial_pos, initial_vel, goal_pos, max_vel, max_acc);
    controller.step();

    system("python3 ../plot_trajectory.py");
}
