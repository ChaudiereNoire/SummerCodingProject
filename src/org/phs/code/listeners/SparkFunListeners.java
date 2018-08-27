package org.phs.code.listeners;

/*
* #%L
* **********************************************************************
* ORGANIZATION  :  Pi4J
* PROJECT       :  Pi4J :: Java Examples
* FILENAME      :  ListenMultipleGpioExample.java
*
* This file is part of the Pi4J project. More information about
* this project can be found here:  http://www.pi4j.com/
* **********************************************************************
* %%
* Copyright (C) 2012 - 2018 Pi4J
* %%
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Lesser Public License for more details.
*
* You should have received a copy of the GNU General Lesser Public
* License along with this program.  If not, see
* <http://www.gnu.org/licenses/lgpl-3.0.html>.
* #L%
*/

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;

/**
* This is a simple listener set up to monitor ports 8 and 7, where two Hall Effect Sensors are positioned
* within four millimeters of a spinning disc that has 4 dipole magnets embedded in it.  It is on the driving 
* end of a 90 RPM (140 RPM Max) electric motor/48:1 gear box combination.  The wheels attached to the motors 
* are 6.5 cm in diameter, or 20.4 cm in circumference.  When this runs with the DAGU encoders, the sensors 
* are on the motor side, making 48 full turns of the disc for each turn of the wheels.  One ideal revolution 
* counts up 394 ticks.
*   
* The distance per tick is thus 20.4 cm / (48 * 8 ticks), or 0.05 cm per tick.
*/
public class SparkFunListeners {
	private long countA;
	private long countB;

	public SparkFunListeners() throws InterruptedException {
	 
		System.out.println("Initializing the GPIO Listener.");
		// create GPIO controller
		final GpioController gpio = GpioFactory.getInstance();

		Pin pinOutA = RaspiPin.GPIO_07;
		Pin pinOutB = RaspiPin.GPIO_08;
		PinPullResistance pullOutA = PinPullResistance.PULL_UP;
		PinPullResistance pullOutB = PinPullResistance.PULL_UP;
		countA = 0;
		countB = 0;

		System.out.println("Provisioning Pins 7 & 8.");

		// provision gpio pin #07 and #08 as input pins with internal pull resistor set to UP
		final GpioPinDigitalInput outA = gpio.provisionDigitalInputPin(pinOutA, pullOutA);
		final GpioPinDigitalInput outB = gpio.provisionDigitalInputPin(pinOutB, pullOutB);

		// unexport the GPIO pin when program exits
		outA.setShutdownOptions(true);
		outB.setShutdownOptions(true);

		System.out.println("Setting up Listener A on Pin 7.");
		// create and register gpio pin listener for out A
		outA.addListener(new GpioPinListenerDigital() {
			@Override
			public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent event) {
				countA += 1;
			}

		});

		
		System.out.println("Setting up Listener B on Pin 8.");
		// create and register gpio pin listener for out B
		outB.addListener(new GpioPinListenerDigital() {
			@Override
			public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent event) {
				countB += 1;
			}

		});

	}
	
	public long[] getCounts() {
		long[] counts = {0,0};
		counts[0] = countA;
		counts[1] = countB;
		return counts;
	}
 
	public long getCountA() {
		return countA;
	}

	public void clearCountA() {
		countA = 0;
	}
 
	public long getCountB() {
		return countB;
	}

	public void clearCountB() {
		countB = 0;
	}
 
}