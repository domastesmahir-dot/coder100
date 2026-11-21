#include "main.h"
#include <algorithm>
#include <format>
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/screen.h"
#include "subsystems.hpp"
#include "recorder.hpp"  // Phase 1 data logger (record-and-play project)
#include "playback.hpp"  // plays recordings as autonomous (record-and-play project)
//#include "autons.cpp"

string print1 = R"(
   ____   _____ ______
  / __ \ / ___// ____/  Robotics
 / / / / \__ \/ __/     72202
/ /_/ / ___/ / /___     
\___\_\/____/_____/     
)";
pros::Distance dist_sensor(3);  // claw auto-close sensor (moved from port 8)
pros::Distance dist_sensor2(21);
pros::Optical left_color_sensor(7);
pros::Optical right_color_sensor(6);
string desired_color = "yellow";
bool prev_press = false;

// Chassis constructor
ez::Drive chassis(
  // Left side: port 19 listed FIRST on purpose. EZ-Template measures each side
  // with the first motor in its list, and port 18's sensor read 0 in every test
  // (speed and distance) while the motor still drove. That broke the active
  // brake (robot swerved left when you let go) and odometry. Was {-18, -19}.
  {-19, -18}, 
  {17, 16}, 

  10,
  2.75,  
  450
);   

// I don't want to go function digging to find what ezlib sets the imu to be so ima just define it again
pros::IMU inertial(10);
pros::Motor L1(-18);
pros::Motor L2(-19);
pros::Motor R1(17);
pros::Motor R2(16);
ez::tracking_wheel vert_tracker(20, 2, 4); 

bool killSwitch = false;

void initialize() {
  lift.set_brake_mode(MOTOR_BRAKE_HOLD);
  // Print our branding over your terminal :D
  ez::ez_template_print();
  // Actually I don't like you doing that >:(
  //pros::screen::print(pros::E_TEXT_SMALL,1,print.c_str());
  pros::delay(500);  
  bool overheated = false;
  string print = "m";
  print = print1;
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {print + "R STD", drive_example},
      //{print + "L STD", left_standard},
      //{print + "R 1PIN", drive_and_turn},
      //{print + "L 1PIN", wait_until_change_speed},
  });
  // Record-and-play: add one selector entry per built-in recording ("REC ..."),
  // after the autons above. Must come right after autons_add. See src/playback.cpp.
  playback_add_to_selector(print);

  // Initialize chassis and auton selector

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}
//This color sort should work anywhere
void color_sort(pros::Optical Sensor, pros::Motor Running_Motor,bool invert,string desired) {
  double hue = 0;
  if(desired=="red" || desired=="Red" || desired=="RED" || desired=="R") {
    hue = 12;
  }
  if(desired=="yellow" || desired=="Yellow" || desired=="YELLOW" || desired=="Y") {
    hue = 50;
  }
  if(desired=="blue" || desired=="Blue" || desired=="BLUE" || desired=="B") {
    hue = 205;
  }
  double color_difference = min({
    abs(360.0-Sensor.get_hue()-hue),
    abs(Sensor.get_hue()-hue),
    abs(360.0-Sensor.get_hue()-hue)
  });
  if(color_difference>32){
    if(invert){
      Running_Motor.move(-80);
    } else {
      Running_Motor.move(80);
    }
  }
  //leftST.move(-127);
}

void disabled() {
  
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}


void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  chassis.pid_tuner_enable();
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
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

// Anti tip numbers
// Basically a poor man's pid
// The graph can be shown with:
// antiTipCoefficient\left((antiTipSquish)(x))\right)^{a_{ntiTipExponent}}
// (LaTeX)
// antiTipCoefficient * ((antiTipSquish) * (x)) ^ (antiTipExponent)
// when x is the roll (or basically the asumith degree)
// Offset is literally just an offset to the imu
int offset = -4;
int antiTipCoefficient = 1.8; //disabled, 1.8 is good currently
int antiTipSquish = 1.7;
int antiTipExponent = 2.6;
int moveChange = 0;

void opcontrol() {
  bool claw_init = true;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  float constant = 0.75;
  uint32_t time = 0;
  //autonomous();
  while (true) {
    if (killSwitch) {
      moveChange = antiTipCoefficient * std::pow((antiTipSquish) * (inertial.get_roll()+offset),antiTipExponent);
    } else {
      moveChange = 0;
    }

    if (master.get_digital(DIGITAL_UP)) {killSwitch = true;}
    if (master.get_digital(DIGITAL_DOWN)) {killSwitch = false;}

    if (claw_init & (dist_sensor.get() > 220)) {
      claw_init = false;
    }
    ez_template_extras();
    
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    
    L1.move(L1.get_target_velocity()+moveChange);
    L2.move(L2.get_target_velocity()+moveChange);
    R1.move(R1.get_target_velocity()+moveChange); 
    R2.move(R2.get_target_velocity()+moveChange);

    if (master.get_digital(DIGITAL_B)) {
      gripper.set_value(false);
      claw_init = false;
      time = pros::millis();
     
    } else if (master.get_digital(DIGITAL_X) || ((dist_sensor.get() < 90 && !claw_init) &&/* !(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < -10) &&*/ (pros::millis() - time)>500)) {
      gripper.set_value(true);
      claw_init = true;
    }
    if (claw_init && (dist_sensor.get() > (100 + constant * (chassis.drive_velocity_left() + chassis.drive_velocity_right()) / 2))) {
      claw_init = false;
    }

    if (master.get_digital(DIGITAL_UP)) {
      Wedge.set_value(true);
    } else if (master.get_digital(DIGITAL_DOWN)) {
      Wedge.set_value(false);
    }
    
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME

    if (master.get_digital(DIGITAL_R2)) {
      lift.move(127);
    } else if (master.get_digital(DIGITAL_R1)) {
      lift.move(-127);
    } else {
      lift.move(0);
    }
    if (master.get_digital(DIGITAL_L2) && !prev_press) {
      if(desired_color=="yellow") {
        desired_color="red";
      } else
      if(desired_color=="red") {
        desired_color="blue";
      } else if(desired_color=="blue") {
        desired_color="red";
      };
      prev_press = true;
    }
    if (!master.get_digital(DIGITAL_L2)) {
      prev_press = false;
    }
    
    /*
    if (master.get_digital(DIGITAL_L1)) {
      left_color_sensor.set_led_pwm(100);
      color_sort(left_color_sensor,rightST,true,desired_color);//idk anymore
      //color_sort(right_color_sensor,rightST,false,desired_color);
    } else {
      left_color_sensor.set_led_pwm(0);
      leftST.move(0);
      rightST.move(0);
    }
    */
    if (master.get_digital(DIGITAL_L1)/* && !master.get_digital(DIGITAL_L1)*/) {
      leftST.move(-127);
      rightST.move(-127);
    } else if (master.get_digital(DIGITAL_L2)/* && !master.get_digital(DIGITAL_L1)*/) {
      leftST.move(127);
      rightST.move(127);
    } else if (true/*!master.get_digital(DIGITAL_L1)*/) {
      leftST.move(0);
      rightST.move(0);
    }

    // Record-and-play project, Phase 1: if a recording is running, save one
    // row of controller + sensor data. Only WATCHES - never moves anything
    // (except stopping the drive while it saves). See src/recorder.cpp.
    recorder_update();
    
}
}
