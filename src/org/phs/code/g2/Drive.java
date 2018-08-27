package org.phs.code.g2;

import org.phs.code.motors.Motor;
import org.phs.code.motors.ReverseMotor;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiBcmPin;
import com.pi4j.io.gpio.RaspiGpioProvider;
import com.pi4j.io.gpio.RaspiPinNumberingScheme;

public class Drive {

	private GpioPinDigitalOutput enableGpioPin;
	private Motor motor1;
	private Motor motor2;
	boolean enabled = false;

	public Drive() {
		
    	Pin enablePin =RaspiBcmPin.GPIO_11;
    	
        GpioFactory.setDefaultProvider(new RaspiGpioProvider(RaspiPinNumberingScheme.BROADCOM_PIN_NUMBERING));
		GpioController gpio = GpioFactory.getInstance();
		enableGpioPin = gpio.provisionDigitalOutputPin(enablePin, "stby");

        motor1 = new GpioMotor(RaspiBcmPin.GPIO_12, RaspiBcmPin.GPIO_24, RaspiBcmPin.GPIO_22);
        motor2 = new ReverseMotor(
             new GpioMotor(RaspiBcmPin.GPIO_13, RaspiBcmPin.GPIO_25, RaspiBcmPin.GPIO_23));
 
        
    }

	public void enable() {
		enableGpioPin.setState(PinState.HIGH);
		enabled = true;
	}

	public void disable() {
		enableGpioPin.setState(PinState.LOW);
		enabled = false;
	}

	public void setSpeeds(int motor1Speed, int motor2Speed) {
		if (!enabled) {
			enable();
		}
		motor1.setSpeed(motor1Speed);
		motor2.setSpeed(motor2Speed);
	}
	
	/*
	 * Tank Drive uses the y-axis of two joysticks to steer the vehicle.  
	 */
	public void tankDrive(double leftMotorSpeed, double rightMotorSpeed) {
		setSpeeds((int) leftMotorSpeed, (int) rightMotorSpeed);
		
	}
	
	public void arcadeDrive() {
		
	}

}