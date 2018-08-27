package org.phs.code.robot;
/*
 * This program was set up to demonstrate some of the coding principles for our summer of code.  We
 * are building a robot with these components and learing to handle reading digital data, processing
 * interrupts, and using sensor data to drive the robot.
 * 
 * RobotMain will be the entry point for the application and will be passed over to the Raspberry Pi
 * and called up by a shell script during the normal startup.
 * 
 * Chassis      https://www.sparkfun.com/products/13301 w/ battery pack, $12.95
 * Breadboard   https://www.sparkfun.com/products/12002, tiny,           $ 4.95
 * Gyro         https://www.sparkfun.com/products/13762, 9 DOF,          $14.95
 * Motors       https://www.sparkfun.com/products/13302 Pack of 2,       $ 3.95
 * Driver       https://www.sparkfun.com/products/14450, Dual,           $ 4.95
 * Encoders     https://www.sparkfun.com/products/12629, Pack of 2,      $12.95
 * Wheels       https://www.sparkfun.com/products/13259, Pack of 2,      $ 2.95
 * 
 * IR Sensors   https://www.sparkfun.com/products/12728, start w/ 1,     $13.95 
 * IR Connector https://www.sparkfun.com/products/8733, start w/ 1,      $ 1.50  
 * 
 * GPIO Usage:
 *  - PIN 01 : 3.3 VDC --> Board Red bar (a side) Power Rail
 *  - PIN 02 : 5.0 VDC --> Board Red bar (g side) Power Rail 
 *  - PIN 03 : GPIO 02 SDA1 (I2C) --> Board 20b --> MPU 9250 SDA
 *  - PIN 04 : 5.0 VDC
 *  - PIN 05 : GPIO 03 SCL1 (I2C) --> Board 19b --> MPU 9250 SCL
 *  - PIN 06 : GND ------> Board Black Bar (g side) Ground
 *  - PIN 07 : GPIO 04
 *  - PIN 08 : GPIO 14 TxD (UART)
 *  - PIN 09 : GND
 *  - PIN 10 : GPIO 15 RxD (UART)
 *  - PIN 11 : GPIO 07 
 *  - PIN 12 : GPIO 18
 *  - PIN 13 : GPIO 27
 *  - PIN 14 : GND
 *  - PIN 15 : GPIO 22 --> Board 24j --> MOTOR DRIVER AI2
 *  - PIN 16 : GPIO 23 --> Board 28j --> MOTOR DRIVER BI2
 *  - PIN 17 : 3.3 VDC
 *  - PIN 18 : GPIO 24 --> Board 25j --> MOTOR DRIVER AI1
 *  - PIN 19 : GPIO 10 MOSI (SPI)
 *  - PIN 20 : GND
 *  - PIN 21 : GPIO 09 MISO (SPI)
 *  - PIN 22 : GPIO 25 --> Board 27j --> MOTOR DRIVER BI1
 *  - PIN 23 : GPIO 11 --> Board 26j --> MOTOR DRIVER STBY
 *  - PIN 24 : GPIO 08 --> Board 15j --> Left Encoder Signal
 *  - PIN 25 : GND
 *  - PIN 26 : GPIO 07 --> Board 13j --> Right Encoder Signal 
 *  - PIN 27 : ID_SD   SDA0 (I2C ID EEPROM)
 *  - PIN 28 : ID_SC   SCL0 (I2C ID EEPROM)
 *  - PIN 29 : GPIO 05
 *  - PIN 30 : GND
 *  - PIN 31 : GPIO 06
 *  - PIN 32 : GPIO 12 --> Board 29j --> MOTOR DRIVER PWMB
 *  - PIN 33 : GPIO 13 --> Board 23j --> MOTOR DRIVER PWMA
 *  - PIN 34 : GND
 *  - PIN 35 : GPIO 19
 *  - PIN 36 : GPIO 16
 *  - PIN 37 : GPIO 26
 *  - PIN 38 : GPIO 20
 *  - PIN 39 : GND
 *  - PIN 40 : GPIO 21
 *  
 *  Patch 
 *  Gyro SparkFun MPU-9250 IMU  
 *  
 *  - 22a GND Blue Bar 
 *  - 21a VDD Red Bar
 *  - 20a SDA --> PIN 03
 *  - 19a SCL --> PIN 05
 *  
 *  Motor Controller
 *  
 *  - 30d GND
 *  - 29d B01 --> Left Motor Red
 *  - 28d B02 --> Left Motor Black
 *  - 27d A02 --> Right Motor Red
 *  - 26d A01 --> Right Motor Black
 *  - 25d GND
 *  - 24d VCC
 *  - 23d VM
 *  
 *  - 30h GND
 *  - 29h PWMB --> GPIO 36 (Pin 31)
 *  - 28h BIN2
 *  - 27h BIN1
 *  - 26h STBY
 *  - 25h AIN1 --> 
 *  - 24h AIN2 -->
 *  - 23h PWMA --> GPIO 23 (Pin 32)
 *   
 */

import org.phs.code.robot.Planner;
import org.phs.code.robot.Logging;
import org.phs.code.i2c.I2CArduino;
import org.phs.code.listeners.SparkFunListeners;
import org.phs.code.nt.TableJoystick;
import org.phs.code.g2.Drive;
import org.phs.code.i2c.I2CCompass;

public class RobotMain {

	public static double[] pose;

	public static void main(String[] args) throws Exception {
		
		// We will keep a ticker, with the time at the start of the main loop captured
		long prevtime = System.currentTimeMillis();
		long currenttime = System.currentTimeMillis();
		
		long leftCount = 0;
		long rightCount = 0;
		long prevLeftCount = 0;
		long prevRightCount = 0;
		
		double[] dTravel = {0, 0, 0 };
		
		int leftSpeed = 0;
		int rightSpeed = 0;
		
        try   {
            Logging.CustomLogger.setup();
        }
        catch (Throwable e) { Logging.logException(e);}
        Logging.consoleLog();

        
		// Initialize Planner element
		Planner planner = new Planner();
		planner.init();
		
		// Initialize NetworkTables and Joystick element
		TableJoystick stick = new TableJoystick();
		
		// Initialize the State Assessment element
		StateAssessment state = new StateAssessment();
		state.init();
		
		// set up the driver subsystem
		Drive drive = new Drive();
		
		// Initialize the sensor elements of the Robot
		//   I2CGyro i2cGyro = new I2CGyro();
		SparkFunListeners listener = new SparkFunListeners();
		listener.clearCountA();
		listener.clearCountB();
		
		// Initialize the sensor elements of the Robot
		//   I2CGyro i2cGyro = new I2CGyro();
		I2CCompass compass = new I2CCompass();
		compass.init();
		
		I2CArduino arduino = new I2CArduino();
		arduino.init();
		
		/*
		 * This is the main loop.  Once the vehicle determines that it is at the
		 * destination, it stops. 
		 */
		double distance = 0;
		while (!(state.atDestination() | (distance  > 100))) {
			
			// We will record time at the start of each loop, saving the value from our last pass
			
			// Report stored values from DAGU Encoders
			prevLeftCount = leftCount;
			prevRightCount = rightCount;
			prevtime = currenttime;
			
			currenttime = System.currentTimeMillis();

			// Get readings from DAGU Encoders
			leftCount = listener.getCountA();
			rightCount = listener.getCountB();
			
		    // Get the current position
			state.computeCurrentPose( currenttime, leftCount, rightCount);
			dTravel = state.getDistanceTravelled();
			
			
			// Get readings from MPU9250 & AK6983
			
			/*
			 * Since we are not at our destination, check to see if the current plan is still
			 * good and continue on course if it is.
			 */
			
		    // Calculate path to current way post
			int motorSpeeds[] = planner.calculateNextMove( state.getCurrentPose(), state.getCurrentTarget());

		    // Get the current way post
			planner.getCurrentWayPost(motorSpeeds);
			
		    //  Set motor speeds to resolve path error
			//leftSpeed = motorSpeeds[0];
			//rightSpeed = motorSpeeds[1];
			double[] values = stick.PollMaster();
			leftSpeed = (int) ((int) 480 * stick.getLeftY());
			rightSpeed = (int) ((int) 480 * stick.getRightY());

			
			Logging.consoleLog("%d,%d,%d,%d,%d,%d,%d,%d", 
					currenttime, currenttime - prevtime, leftCount, leftCount - prevLeftCount, 
					rightCount, rightCount - prevRightCount, leftSpeed, rightSpeed);
			
			// drive.setSpeeds(leftSpeed, rightSpeed);
			drive.tankDrive(leftSpeed, rightSpeed);
			
			state.setCurrentPose(motorSpeeds[2], motorSpeeds[3], 0.0, 0.0);

			/*
			 * Sleep for a few milliseconds to allow the worker threads to keep up.  Here we try to make every cycle
			 * take about 67 milliseconds to complete.  This was done to make it easier to spot anomalies in the data.
			 */
			
			long sleepTime = 34 - (System.currentTimeMillis() - currenttime);
			if (sleepTime > 0) {
				Thread.sleep(sleepTime);
			}
			
			distance = dTravel[0];
			
		}

		leftSpeed = 0;
		rightSpeed = 0;
		
		drive.setSpeeds(leftSpeed, rightSpeed);

		long sleepTime = 34 - (System.currentTimeMillis() - currenttime);
		if (sleepTime > 0) {
			Thread.sleep(sleepTime);
		}
		
		drive.disable();

		
		
		int iter = 1;
		while (iter < 50) {
			
			// We will record time at the start of each loop, saving the value from our last pass
			
			// Report stored values from DAGU Encoders
			prevLeftCount = leftCount;
			prevRightCount = rightCount;
			prevtime = currenttime;
			
			currenttime = System.currentTimeMillis();

			// Get readings from DAGU Encoders
			leftCount = listener.getCountA();
			rightCount = listener.getCountB();
			
			Logging.consoleLog("%d,%d,%d,%d,%d,%d,%d,%d", 
					currenttime, currenttime - prevtime, leftCount, leftCount - prevLeftCount, 
					rightCount, rightCount - prevRightCount, leftSpeed, rightSpeed);
			
			/*
			 * Sleep for a few milliseconds to allow the worker threads to keep up.  Here we try to make every cycle
			 * take about 67 milliseconds to complete.  This was done to make it easier to spot anomalies in the data.
			 */
			sleepTime = 34 - (System.currentTimeMillis() - currenttime);
			if (sleepTime > 0) {
				Thread.sleep(sleepTime);
			}
			iter += 1;
		}
		
		System.out.println("Loop completed.");
	}
}