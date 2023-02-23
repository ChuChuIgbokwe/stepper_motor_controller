#include <iostream>
#include "src/StepperController.h"
#include "src/ugly_no_class.h"
#include <math.h>
#include <fstream>
#include "src/brand_new.h"
//#include "gpt_v3.h"
#include "gpt_v4.h"

int main() {
//    stepper_motor_controller stepperMotorController(150, -50);
//    stepperMotorController.printValues();
//    stepperMotor(-150, -50, 850, 75, 10);
//    java_stepper(-150, -50, 850, 75, 10);
//    stepper_motor_brand_new(-150, -50, 850, 75, 10);
//    smc(-150, -50, 550, 75, 10);
//    smc(-150, -50, -1150, 75, 10);
//    StepperController SMC(150, -50);
//    SMC.set_goal(850, 75, 10);
//    SMC.step()
//StepperController stepperController(150, -50);
//stepperController.set_goal(850,75,10);
//stepperController.step();
    StepperController stepperController(-150, -50);
    stepperController.set_goal(550, 75, 10);
    stepperController.step();
}
