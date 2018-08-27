package org.phs.code.motors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.phs.code.g2.GpioMotor;

public class SleepUtil {
	private static final Logger LOG = LoggerFactory.getLogger(GpioMotor.class);

	public static void sleepSeconds(double seconds) {
		try {
			Thread.sleep((long) (seconds * 1000));
		} catch (InterruptedException e) {
			LOG.warn("sleep() InterruptedException", e);
		}
	}
	
	private SleepUtil() {
	}

}