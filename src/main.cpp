#include "main.h"
#include "okapi/api.hpp"
#include "HolonomicLib/API.hpp"
using namespace okapi;

okapi::IMU imu(4);

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
	.withMotors(
		7,	//top left
		10,	//top right
		9,	//bottom right
		8	///bottom left
	)
	.withSensors(
		RotationSensor{1},	//Right sensor
		RotationSensor{2},	//left sensor
		RotationSensor{3}	//middle sensor
	)
	.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
	.buildOdometry();

// X-Drive controlller creation
std::shared_ptr<AsyncHolonomicChassisController> controller = AsyncHolonomicChassisControllerBuilder(chassis)
	.withDistGains(
		//tracking wheel diameter, track width, middle encoder dist, diameter
		{0.05, 0.0, 0.00065, 0.0}
	)
	.withTurnGains(
		{0.05, 0.0, 0.00065, 0.0}
	)
	.build();

std::shared_ptr<XDriveModel> model = std::static_pointer_cast<XDriveModel> (chassis->getModel());
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.calibrate();

    while (imu.isCalibrating()) {
        pros::delay(10);
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller controller = Controller();

	while (true) {
		auto heading = imu.get() * okapi::degree;

		model->fieldOrientedXArcade(
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::leftX),
			controller.getAnalog(ControllerAnalog::rightX),
			heading,
			0.05
		);

		pros::delay(20);
	}
}
