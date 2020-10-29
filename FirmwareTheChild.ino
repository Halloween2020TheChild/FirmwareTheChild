#include <Arduino.h>
#include <RBE1001Lib.h>
#include "Motor.h"
#include "Rangefinder.h"
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>
#include <Esp32WifiManager.h>
#include "wifi/WifiManager.h"
#include "WebPage.h"
#include <Timer.h>
#include <lx16a-servo.h>
LX16ABus servoBus;
LX16AServo servo(&servoBus, 1);
LX16AServo servo2(&servoBus, 2);
LX16AServo servo3(&servoBus, 3);

enum walkState {
	stopped = 0,
	stopping = 1,
	leftFootMove = 2,
	waitingForLeftToFinish = 3,
	rightFootMove = 4,
	waitingForRightToFinish = 5,
	waitForLeftHalfCycle,
	waitForRightHalfCycle
};

enum walkState state = stopped;
float rightDelt = 0;
float leftDelt = 0;

// https://wpiroboticsengineering.github.io/RBE1001Lib/classMotor.html
Motor left_motor;
Motor right_motor;
// https://wpiroboticsengineering.github.io/RBE1001Lib/classRangefinder.html

WebPage control_page;

WifiManager manager;

Timer dashboardUpdateTimer;  // times when the dashboard should update

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
	if (x > in_max)
		return out_max;
	if (x < in_min)
		return out_min;

	float divisor = (in_max - in_min);
	if (divisor == 0) {
		return -1; //AVR returns -1, SAM returns 0
	}
	return (x - in_min) * (out_max - out_min) / divisor + out_min;
}
/*
 * This is the standard setup function that is called when the ESP32 is rebooted
 * It is used to initialize anything that needs to be done once.
 * In this example, it sets the Serial Console speed, initializes the web server,
 * sets up some web page buttons, resets some timers, and sets the initial state
 * the robot should start in
 */
int inc = 0;

void setup() {
	servoBus.begin(&Serial2, 17, // on TX pin 10
			19); // use pin 2 as the TX flag for buffer
	Serial.begin(115200);
	servoBus.retry = 1; // enforce synchronous real time
	//servoBus.debug(true);
	Serial.println("Beginning Coordinated Servo Example");
	servo.disable();
	servo2.disable();
	servo3.disable();

	manager.setup();
	while (manager.getState() != Connected) {
		manager.loop();
		delay(1);
	}
	Motor::allocateTimer(0); // used by the DC Motors
	ESP32PWM::allocateTimer(1); // Used by servos
	control_page.initalize();
	// pin definitions https://wpiroboticsengineering.github.io/RBE1001Lib/RBE1001Lib_8h.html#define-members
	right_motor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA,
	MOTOR_RIGHT_ENCB);
	left_motor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA,
	MOTOR_LEFT_ENCB);

	dashboardUpdateTimer.reset(); // reset the dashbaord refresh timer
	Serial.println("servo.readLimits()");
	servo.calibrate(0, -4500, 4500);
	Serial.println("servo2.readLimits()");
	servo2.calibrate(0, -1000, 4500);
	Serial.println("servo3.readLimits()");
	servo3.calibrate(0, -1000, 4000);
	servo2.move_time_and_wait_for_sync(0, 0);
	servo3.move_time_and_wait_for_sync(0, 0);
	servo.move_time_and_wait_for_sync(0, 0);

}

/*
 * this is the state machine.
 * You can add additional states as desired. The switch statement will execute
 * the correct code depending on the state the robot is in currently.
 * For states that require timing, like turning and straight, they use a timer
 * that is zeroed when the state begins. It is compared with the number of
 * milliseconds the robot should reamain in that state.
 */
void runStateMachine() {

	int sliderMode = (int) (control_page.getSliderValue(0) * 3.0);
	float x = control_page.getJoystickX();
	float y = control_page.getJoystickY();
	float distance = 100;
	float time = 800;

	switch (sliderMode) {
	case 0:
		if (fabs(x) < 0.01 && fabs(y) < 0.01) {
			state = stopping;
		}
		leftDelt = ((x - y) * distance);
		rightDelt = ((x + y) * distance);
		switch (state) {
		case stopped:
			if (fabs(x) > 0.01 || fabs(y) > 0.01) {
				state = leftFootMove;
			}
			break;
		case stopping:
			left_motor.setSpeed(0);
			right_motor.setSpeed(0);
			servo2.move_time_and_wait_for_sync(0, time*2/3);
			servo3.move_time_and_wait_for_sync(0, time*2/3);
			servo.move_time_and_wait_for_sync(0, time*2/3);
			servoBus.move_sync_start();
			state = stopped;
			break;
		case leftFootMove:
			left_motor.setSetpointWithBezierInterpolation(
					left_motor.getCurrentDegrees() + leftDelt, time, 0.2, 1);
			servo2.move_time_and_wait_for_sync(0, time/3);
			servo3.move_time_and_wait_for_sync(0, time/3);
			servo.move_time_and_wait_for_sync(1500*x, time/3);
			servoBus.move_sync_start();
			state = waitForLeftHalfCycle;
			break;
		case waitForLeftHalfCycle:
			if (left_motor.getInterpolationUnitIncrement() >= 0.5){
				state = waitingForLeftToFinish;
				servo2.move_time_and_wait_for_sync(1000, time/3);
				servo3.move_time_and_wait_for_sync(-1000, time/3);
				//servo.move_time_and_wait_for_sync(0, time/3);
				servoBus.move_sync_start();
			}
			break;
		case waitingForLeftToFinish:
			if (left_motor.getInterpolationUnitIncrement() >= 1)
				state = rightFootMove;
			break;
		case rightFootMove:
			right_motor.setSetpointWithBezierInterpolation(
					right_motor.getCurrentDegrees() + rightDelt, time, 0.2, 1);
			servo2.move_time_and_wait_for_sync(0, time/3);
			servo3.move_time_and_wait_for_sync(0, time/3);
			servo.move_time_and_wait_for_sync(-1500*x, time/3);
			servoBus.move_sync_start();
			state = waitForRightHalfCycle;
			break;
		case waitForRightHalfCycle:
			if (right_motor.getInterpolationUnitIncrement() >= 0.5){
				state = waitingForRightToFinish;
				servo2.move_time_and_wait_for_sync(1000, time/3);
				servo3.move_time_and_wait_for_sync(-1000, time/3);
				//servo.move_time_and_wait_for_sync(0, time/3);
				servoBus.move_sync_start();
			}
			break;
		case waitingForRightToFinish:
			if (right_motor.getInterpolationUnitIncrement() >= 1)
				state = leftFootMove;
			break;
		}

		break;
	case 1:
		servo2.move_time_and_wait_for_sync(
				-fmap(control_page.getJoystickX(), -1, 1, -1000, 4500), 0);
		servo3.move_time_and_wait_for_sync(
				fmap(control_page.getJoystickX(), -1, 1, -1000, 4500), 0);
		servo.move_time_and_wait_for_sync(
				fmap(control_page.getJoystickY(), -1, 1, -4500, 4500), 0);
		servoBus.move_sync_start();
		break;
	case 2:
		servo2.move_time_and_wait_for_sync(0, 0);
		servo3.move_time_and_wait_for_sync(
				fmap(control_page.getJoystickX(), -1, 1, -1000, 4500), 0);
		servo.move_time_and_wait_for_sync(
				fmap(control_page.getJoystickY(), -1, 1, -4500, 4500), 0);
		servoBus.move_sync_start();
		break;

	}

}

/*
 * This function updates all the dashboard status that you would like to see
 * displayed periodically on the web page. You are free to add more values
 * to be displayed to help debug your robot program by calling the
 * "setValue" function with a name and a value.
 */

uint32_t packet_old = 0;
void updateDashboard() {
	// This writes values to the dashboard area at the bottom of the web page
	if (dashboardUpdateTimer.getMS() > 100) {
		control_page.setValue("walking state", state);
		control_page.setValue("walking right", rightDelt);
		control_page.setValue("walking left", leftDelt);
		control_page.setValue("packets from Web to ESP",
				control_page.rxPacketCount);
		control_page.setValue("packets to Web from ESP",
				control_page.txPacketCount);
		control_page.setValue("slider", control_page.getSliderValue(0) * 100);

		control_page.setValue("Left Encoder Degrees",
				left_motor.getCurrentDegrees());
		control_page.setValue("Left Effort", left_motor.getEffort());
		control_page.setValue("Left Encoder Degrees/sec",
				left_motor.getDegreesPerSecond());

		control_page.setValue("Right Encoder Degrees",
				right_motor.getCurrentDegrees());
		control_page.setValue("Right  Effort", right_motor.getEffort());
		control_page.setValue("Right Encoder Degrees/sec",
				right_motor.getDegreesPerSecond());

		dashboardUpdateTimer.reset();
	}
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * once the ESP32 is started. In here we run the state machine, update the
 * dashboard data, and handle any web server requests.
 */

void loop() {

	manager.loop();
	runStateMachine();  // do a pass through the state machine
	if (manager.getState() == Connected)  // only update if WiFi is up
		updateDashboard();  // update the dashboard values
}
