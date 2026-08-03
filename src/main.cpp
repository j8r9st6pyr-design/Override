#include "lemlib/api.hpp"
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"
#include <iostream>
#include "main.h"
#include "liblvgl/lvgl.h"

int auton_select = 0;

static void btnm_event_cb(lv_event_t * e) {
    lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
    if(lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        uint32_t id = lv_buttonmatrix_get_selected_button(obj);
        const char * txt = lv_buttonmatrix_get_button_text(obj, id);

        printf("Pressed: %s\n", txt);
        if (txt && strcmp(txt, "Left") == 0) {
          auton_select = 1;
        }
        else if (txt && strcmp(txt, "Right") == 0) {
            auton_select = 2;
        }
        else if (txt && strcmp(txt, "Skills")){
          auton_select = 0;
        }
    }
}

	pros::Controller controller(pros::E_CONTROLLER_MASTER);
  //left side motors
  pros::Motor Left_Half({-3}, pros::MotorGearset::red);
  pros::Motor Left_front({-4}, pros::MotorGearset::blue);
  pros::Motor left_back({-2}, pros::MotorGearset::blue);
  //right side motors
  pros::Motor Right_Half({7}, pros::MotorGearset::blue);
  pros::Motor Right_front({6}, pros::MotorGearset::blue);
  pros::Motor right_back({8}, pros::MotorGearset::blue);
	pros::MotorGroup left_mg({-2, -3, -4});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({6, 7, 8});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
  pros::MotorGroup lift{(-1, 10)};
  pros::adi::Pneumatics claw('A', true);

// drivetrain settings

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(5);
pros::Distance distance_sensor(2);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);

// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
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
		//pros::lcd::set_text(2, "I was pressed!");
	} else {
		//pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

static const char * btn_map[] = {
    "Left ", "Right", "\n",
    "Skills", "\n",
    ""
};

lv_obj_t * btnm;
  void lvgl_task(void *param) {
    while(true) {
        lv_timer_handler();   // processes button presses + events
        pros::delay(5);
    }
}
void initialize() {


    btnm = lv_buttonmatrix_create(lv_screen_active());
    lv_buttonmatrix_set_map(btnm, btn_map);

    lv_obj_set_size(btnm, 300, 150);
    lv_obj_center(btnm);
    lv_obj_set_y(btnm, -60);
    lv_obj_add_event_cb(btnm, btnm_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    pros::Task lvglTask(lvgl_task);

    auto& logger = mvlib::Logger::getInstance();
  mvlib::setOdom(&chassis);
  logger.setRobot({
  .leftDrivetrain = &left_mg,
  .rightDrivetrain = &right_mg
});
  logger.start();
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
void raise_level(int amount, int goal = 0){
  if (amount < 0) {
    amount = -amount;
    for (int i = 0; i < amount; i++) {
      int id = distance_sensor.get();
    while (distance_sensor.get() > id - 334) {
      lift.move(-127); // lower lift
      pros::delay(10);
    }
  }
}
else if (amount = 0){
  int height = 0;
  if (goal == 1){ //alliance specific goals
    height = 88;
  }
  else if (goal == 2) {
    height = 150; // neutral goals
  }
  else if (goal == 0) {
    height = 10; // floor
  } 
  else {
    height = 225; // center goal
  }
  while (distance_sensor.get() <= height + 1 && distance_sensor.get() >= height - 1){
  if (distance_sensor.get() < height) {
 lift.move(127); // raise lift
  }
  else if (distance_sensor.get() > height) {
    lift.move(-127); // lower lift
  }
  pros::delay(10);

  for (int i = 0; i < amount; i++) {
    int id = distance_sensor.get();
    while (distance_sensor.get() < id + 334) {
    lift.move(127); // raise lift
    pros::delay(10);
    }
  }
}
    lift.move(0);
  }
}

void clamp(bool state){
  if (state) {
    claw.extend();
  }
  else{
    claw.retract();
  }
}
void Match_autonomous_Right() {
  chassis.setPose(0,0,0);
  chassis.moveToPoint(0,-2,500);
  pros::delay(100);
  lift.move(127); // raise lift
  pros::delay(500);
  chassis.moveToPoint(0,0,500);
  lift.move(-127); // stop lift
  pros::delay(350);
  lift.move(0);
  chassis.moveToPoint(-12, 12, 1000);
  raise_level(0, 2);
  raise_level(1);
  pros::delay(400);
  clamp(false);
  chassis.moveToPoint(-9, 12, 1000, {.forwards = false});
  raise_level(0,0);
  chassis.moveToPoint(-10, 6, 1000);
  chassis.moveToPoint(-10, 0, 1000, {.forwards = false});
  chassis.turnToPoint(-12, 0, 1000);
  chassis.moveToPoint(-12, 0, 1000);
  clamp(true);
  chassis.turnToPoint(-12, 12, 1000);
  chassis.moveToPoint(-12, 12, 1000);
  pros::delay(300);
  raise_level(0,2); // raise lift
  raise_level(1);
  clamp(false);
  chassis.moveToPoint(-12, 9, 1000);
  raise_level(0,0); // lower lift
  pros::delay(300);
  chassis.moveToPoint(-14, 6, 750);
  chassis.moveToPoint(-7, 6, 1000);
  chassis.moveToPoint(0, 24, 1000);
  clamp(true);
  //pick pin and holder
  chassis.turnToPoint(0, 12, 1000);
  chassis.moveToPoint(0, 12, 1000, {.forwards = false});
  pros::delay(500);
  chassis.moveToPoint(12, 12, 1000);
  //place pin and holder
  chassis.moveToPoint(9, 6, 1000, {.forwards = false});
  chassis.turnToPoint(12, 0, 1000, {.forwards = false});
  chassis.moveToPoint(12, 0, 1000, {.forwards = false});
  // pick pin and holder
  chassis.moveToPoint(12, 12, 1000);
  // place pin and holder
  chassis.moveToPoint(14, 8, 1000, {.forwards = false});
  chassis.moveToPoint(18, 10, 1000, {.forwards = false});
  chassis.moveToPoint(24, 12, 1000, {.forwards = false});
  // pick pin and holder
  chassis.moveToPoint(24, 24, 1000);
  //place pin and holder
  chassis.moveToPoint(26, 20, 1000, {.forwards = false});
  chassis.turnToPoint(36, 24, 1000, {.forwards = false});
  chassis.moveToPoint(36, 20, 1000, {.forwards = false});
  chassis.moveToPoint(36, 24, 1000, {.forwards = false});
  //pick pin and holder
  chassis.moveToPoint(24, 24, 1000);
  //place pin and holder
}

void match_auto_left(){
chassis.setPose(0,0,0);
  chassis.moveToPoint(0,-2,500);
    pros::delay(100);
  lift.move(127); // raise lift
  pros::delay(500);
  chassis.moveToPoint(0,0,500);
  lift.move(-127); // stop lift
  pros::delay(350);
  lift.move(0);
  chassis.moveToPoint(12, 12, 1000);
  // place pin and holder
  chassis.moveToPoint(12, 0, 1000, {.forwards = false});
  //pick up pin and hourglass
  chassis.moveToPoint(12, 12, 1000);
  //place pin and holder
  chassis.moveToPoint(14, 6, 750);
  chassis.moveToPoint(7, 6, 1000);
  chassis.moveToPoint(0, 24, 1000);
  //pick pin and holder
  chassis.turnToPoint(0, 12, 1000);
  chassis.moveToPoint(0, 12, 1000, {.forwards = false});
  pros::delay(500);
  chassis.moveToPoint(-12, 12, 1000);
  //place pin and holder
  chassis.moveToPoint(-9, 6, 1000, {.forwards = false});
  chassis.turnToPoint(-12, 0, 1000, {.forwards = false});
  chassis.moveToPoint(-12, 0, 1000, {.forwards = false});
  // pick pin and holder
  chassis.moveToPoint(-12, 12, 1000);
  // place pin and holder
  chassis.moveToPoint(-14, 8, 1000, {.forwards = false});
  chassis.moveToPoint(-18, 10, 1000, {.forwards = false});
  chassis.moveToPoint(-24, 12, 1000, {.forwards = false});
  // pick pin and holder
  chassis.moveToPoint(-24, 24, 1000);
  //place pin and holder
  chassis.moveToPoint(-26, 20, 1000, {.forwards = false});
  chassis.turnToPoint(-36, 24, 1000, {.forwards = false});
  chassis.moveToPoint(-36, 20, 1000, {.forwards = false});
  chassis.moveToPoint(-36, 24, 1000, {.forwards = false});
  //pick pin and holder
  chassis.moveToPoint(-24, 24, 1000);
  //place pin and holder
}
void Skills_auton(){

}

void autonomous(){

pros::delay(1000);
if (auton_select == 0){
  Skills_auton();
}
else if (auton_select == 1){
  Match_autonomous_Right();
}
else if (auton_select == 2){
  match_auto_left();
}
}

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
      while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
          lift.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
          lift.move(-127);
        }
        else{
          lift.brake();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
          raise_level(1);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
          raise_level(-1);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
          claw.extend();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
          claw.retract();
        }
        // delay to save resources
        pros::delay(25);
    }
}