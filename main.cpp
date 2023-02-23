#include <iostream>
#include "src/StepperController.h"
#include <cstdlib>
#include <vector>
#include <string>

////    StepperController stepperController(-150, -50, 550, 75, 10);
////    StepperController stepperController(-150, -50);
////    stepperController.set_goal(550, 75, 10);
////    stepperController.step();



// Define a helper function to parse command line arguments
template <typename T>
bool getCmdOption(const std::vector<std::string>& args, const std::string& option, T& value) {
    auto iter = std::find(args.begin(), args.end(), option);
    if (iter != args.end() && ++iter != args.end()) {
        try {
            value = std::stoi(*iter);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Invalid argument for " << option << ": " << *iter << "\n";
            return false;
        }
    }
    return false;
}

int main(int argc, char* argv[])
{
    // Define command line options
    int initial_pos = 0;
    int initial_vel = 0;
    int goal_pos = 0;
    int max_vel = 0;
    int max_acc = 0;
    bool show_help = false;

    // Parse command line arguments
    std::vector<std::string> args(argv + 1, argv + argc);
    for (const auto& arg : args) {
        if (arg == "-h" || arg == "--help") {
            show_help = true;
        } else if (!getCmdOption(args, "--initial-pos", initial_pos) ||
                   !getCmdOption(args, "--initial-vel", initial_vel) ||
                   !getCmdOption(args, "--goal-pos", goal_pos) ||
                   !getCmdOption(args, "--max-vel", max_vel) ||
                   !getCmdOption(args, "--max-acc", max_acc)) {
            show_help = true;
            break;
        }
    }

    // Print help message and exit if requested or if command line arguments are invalid
    if (show_help) {
        std::cout << "Usage: " << argv[0] << " [--initial-pos <int>] [--initial-vel <int>] [--goal-pos <int>] [--max-vel <int>] [--max-acc <int>]\n";
        return 0;
    }

    // Set up stepper controller and perform motor movement
    StepperController controller(initial_pos, initial_vel, goal_pos, max_vel, max_acc);
    controller.step();
//

    // Graph position over time if everything looks good. This prevents the last successful plot from being displayed
    // as the csv file hasn't been overwritten yet
    bool sanity_check_flag = controller.isSanityCheckFlag();
    if(sanity_check_flag) {
        system("python3 ../plot_trajectory.py");
    }
    return 0;
}
