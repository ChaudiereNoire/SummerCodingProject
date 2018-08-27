package org.phs.code.g2;

/*
 * Pololu G2 High Power Motor Driver 22A, 6.5V-30V for Raspberry Pi 
 * 
 * Motor A:
 *   PWM M1 = GPIO #12
 *   FLT M1 = GPIO #5
 *   SLP M1 = GPIO #22
 *   DIR M1 = GPIO #24
 *   
 * Motor B:
 *   PWM M2 = GPIO #13
 *   FLT M2 = GPIO #6
 *   SLP M2 = GPIO #23
 *   DIR M2 = GPIO #25  
 */


import static com.pi4j.wiringpi.Gpio.PWM_MODE_MS;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.GpioPinPwmOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.wiringpi.Gpio;

import org.phs.code.motors.Motor;

public class GpioMotor extends Motor {
	// private static final Logger LOG = LoggerFactory.getLogger(Motor.class);

	/**
	 * Motor speeds for this library are specified as numbers between -MAX_SPEED and MAX_SPEED, inclusive.
	 */
	public static final int MAX_SPEED = 480; // 19.2 MHz / 2 / 480 = 20 kHz

	private GpioPinPwmOutput pwmGpioPin;
	private GpioPinDigitalOutput directionGpioPin1;
	private GpioPinDigitalOutput directionGpioPin2;
	public GpioMotor(Pin pwmPin, Pin directionPin1, Pin directionPin2) {
		GpioController gpio = GpioFactory.getInstance();

		Gpio.pwmSetMode(PWM_MODE_MS);
		Gpio.pwmSetRange(MAX_SPEED);
		Gpio.pwmSetClock(2);

		pwmGpioPin = gpio.provisionPwmOutputPin(pwmPin, "pwm");
		pwmGpioPin.setPwmRange(MAX_SPEED);

		directionGpioPin1 = gpio.provisionDigitalOutputPin(directionPin1, "left");
		directionGpioPin2 = gpio.provisionDigitalOutputPin(directionPin2, "right");

		// setShutdownOptions() on (all of, entire) gpio, not just e.g. the directionGpioPin2
		gpio.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		
	}

	public GpioMotor(Pin pwmPin, Pin directionPin1, Pin directionPin2, Pin enablePin) {
		GpioController gpio = GpioFactory.getInstance();

		Gpio.pwmSetMode(PWM_MODE_MS);
		Gpio.pwmSetRange(MAX_SPEED);
		Gpio.pwmSetClock(2);

		pwmGpioPin = gpio.provisionPwmOutputPin(pwmPin, "pwm");
		pwmGpioPin.setPwmRange(MAX_SPEED);

		directionGpioPin1 = gpio.provisionDigitalOutputPin(directionPin1, "left");
		directionGpioPin2 = gpio.provisionDigitalOutputPin(directionPin2, "right");

		// setShutdownOptions() on (all of, entire) gpio, not just e.g. the directionGpioPin2
		gpio.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
	}


	@Override
	public void setSpeed(int speed) {

		boolean stateIN1 = false;
		boolean stateIN2 = false;
		if (speed == 0) {
			stateIN1 = false;
			stateIN2 = false;
		} else if (speed < 0) {
			speed = -speed;
			stateIN1 = true;
			stateIN2 = false;
		} else {
			stateIN1 = false;
			stateIN2 = true;
		}

		if (speed > MAX_SPEED) {
			speed = MAX_SPEED;
		} else if (speed < -MAX_SPEED) {
			speed = -MAX_SPEED;
		}

		System.out.printf("Current counter values - Speed %d.\n\r", speed);

		// Set the Pin States for each side
		directionGpioPin1.setState(stateIN1);
		directionGpioPin2.setState(stateIN2);
		pwmGpioPin.setPwm(speed);
	}
	
}