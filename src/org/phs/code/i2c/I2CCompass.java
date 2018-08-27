package org.phs.code.i2c;

/*
 * #%L
 * **********************************************************************
 * ORGANIZATION  :  Pi4J
 * PROJECT       :  Pi4J :: Java Examples
 * FILENAME      :  I2CExample.java
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

import java.io.IOException;
import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.io.i2c.I2CFactory.UnsupportedBusNumberException;
import com.pi4j.platform.PlatformAlreadyAssignedException;
import com.pi4j.util.Console;

/**
 * This example code demonstrates how to perform simple I2C
 * communication on the Raspberry Pi.  For this example we will
 * connect to a 'MPU9250' IMU.
 *
 * Data Sheet:
 * https://www.robotshop.com/media/files/pdf/MPU9250REV1.0-Specification.pdf
 *
 * You should get something similar printed in the console
 * when executing this program:
 *
 * > <--Pi4J--> I2C Example ... started.
 * > ... reading ID register from MPU9250
 * > MPU9250 ID = 0x68 (should be 0x68)
 * > ... powering up MPU9250
 * > ... reading DATA registers from MPU9250
 * > MPU9250 DATA 0 = 0x1e
 * > MPU9250 DATA 1 = 0x04
 * > ... powering down MPU9250
 * > Exiting bananapi.I2CExample
 *
 *
 * @author Robert Savage
 */
public class I2CCompass {
	
	public I2CDevice mpudevice;
	public I2CDevice magdevice;

    // MPU9250 I2C address
    private static final int MPU9250_ADDR = 0x68; 

    // AK8963 I2C address
    public static final int AK8963_ADDR = 0x0C; 

    // MPU9250 registers
    public static final byte MPU9250_XG_ST_DATA = (byte) 0x00;
    public static final byte MPU9250_YG_ST_DATA = (byte) 0x01;
    public static final byte MPU9250_ZG_ST_DATA = (byte) 0x02;

    public static final byte MPU9250_XA_ST_DATA = (byte) 0x0D;
    public static final byte MPU9250_YA_ST_DATA = (byte) 0x0E;
    public static final byte MPU9250_ZA_ST_DATA = (byte) 0x0F;

    public static final byte MPU9250_XG_OFFSET_H = (byte) 0x13;
    public static final byte MPU9250_XG_OFFSET_L = (byte) 0x14;
    public static final byte MPU9250_YG_OFFSET_H = (byte) 0x15;
    public static final byte MPU9250_YG_OFFSET_L = (byte) 0x16;
    public static final byte MPU9250_ZG_OFFSET_H = (byte) 0x17;
    public static final byte MPU9250_ZG_OFFSET_L = (byte) 0x18;

    public static final byte MPU9250_SMPLRT_DIV = (byte) 0x19;
    public static final byte MPU9250_CONFIG_FIFO = (byte) 0x1A;

    public static final byte MPU9250_DLPF_CFG = (byte) 0x1B;
    public static final byte MPU9250_ACCEL_FS_SEL = (byte) 0x1C;
    public static final byte MPU9250_ACCEL_FCHOICE_B = (byte) 0x1D;
    public static final byte MPU9250_LP_ACCEL_ODR = (byte) 0x1E;
    public static final byte MPU9250_WOM_THR = (byte) 0x1F;
    public static final byte MPU9250_FIFO_EN = (byte) 0x23;
    public static final byte MPU9250_I2C_MST_CTRL = (byte) 0x24;
    public static final byte MPU9250_I2C_SLV0_ADDR = (byte) 0x25;
    public static final byte MPU9250_I2C_SLV0_REG = (byte) 0x26;
    public static final byte MPU9250_I2C_SLV0_CTRL = (byte) 0x27;
    public static final byte MPU9250_I2C_SLV1_ADDR = (byte) 0x28;
    public static final byte MPU9250_I2C_SLV1_REG = (byte) 0x29;
    public static final byte MPU9250_I2C_SLV1_CTRL = (byte) 0x2A;
    public static final byte MPU9250_I2C_SLV2_ADDR = (byte) 0x2B;
    public static final byte MPU9250_I2C_SLV2_REG = (byte) 0x2C;
    public static final byte MPU9250_I2C_SLV2_CTRL = (byte) 0x2D;
    public static final byte MPU9250_I2C_SLV3_ADDR = (byte) 0x2E;
    public static final byte MPU9250_I2C_SLV3_REG = (byte) 0x2F;
    public static final byte MPU9250_I2C_SLV3_CTRL = (byte) 0x30;
    public static final byte MPU9250_I2C_SLV4_ADDR = (byte) 0x31;
    public static final byte MPU9250_I2C_SLV4_REG = (byte) 0x32;
    public static final byte MPU9250_I2C_SLV4_DO = (byte) 0x33;
    public static final byte MPU9250_I2C_SLV4_CTRL = (byte) 0x34;
    public static final byte MPU9250_I2C_SLV4_DI = (byte) 0x35;
    public static final byte MPU9250_I2C_MST_STATUS = (byte) 0x36;
    public static final byte MPU9250_INT_PIN_CFG = (byte) 0x37;
    public static final byte MPU9250_INT_ENABLE = (byte) 0x38;
    public static final byte MPU9250_INT_STATUS = (byte) 0x3A;
    public static final byte MPU9250_ACCEL_XOUT_H = (byte) 0x3B;
    public static final byte MPU9250_ACCEL_XOUT_L = (byte) 0x3C;
    public static final byte MPU9250_ACCEL_YOUT_H = (byte) 0x3D;
    public static final byte MPU9250_ACCEL_YOUT_L = (byte) 0x3E;
    public static final byte MPU9250_ACCEL_ZOUT_H = (byte) 0x3F;
    public static final byte MPU9250_ACCEL_ZOUT_L = (byte) 0x40;
    public static final byte MPU9250_TEMP_OUT_H = (byte) 0x41;
    public static final byte MPU9250_TEMP_OUT_L = (byte) 0x42;
    public static final byte MPU9250_GYRO_XOUT_H = (byte) 0x43;
    public static final byte MPU9250_GYRO_XOUT_L = (byte) 0x44;
    public static final byte MPU9250_GYRO_YOUT_H = (byte) 0x45;
    public static final byte MPU9250_GYRO_YOUT_L = (byte) 0x46;
    public static final byte MPU9250_GYRO_ZOUT_H = (byte) 0x47;
    public static final byte MPU9250_GYRO_ZOUT_L = (byte) 0x48;
    public static final byte MPU9250_EXT_SENS_DATA_00 = (byte) 0x49;
    public static final byte MPU9250_EXT_SENS_DATA_01 = (byte) 0x4A;
    public static final byte MPU9250_EXT_SENS_DATA_02 = (byte) 0x4B;
    public static final byte MPU9250_EXT_SENS_DATA_03 = (byte) 0x4C;
    public static final byte MPU9250_EXT_SENS_DATA_04 = (byte) 0x4D;
    public static final byte MPU9250_EXT_SENS_DATA_05 = (byte) 0x4E;
    public static final byte MPU9250_EXT_SENS_DATA_06 = (byte) 0x4F;
    public static final byte MPU9250_EXT_SENS_DATA_07 = (byte) 0x50;
    public static final byte MPU9250_EXT_SENS_DATA_08 = (byte) 0x51;
    public static final byte MPU9250_EXT_SENS_DATA_09 = (byte) 0x52;
    public static final byte MPU9250_EXT_SENS_DATA_10 = (byte) 0x53;
    public static final byte MPU9250_EXT_SENS_DATA_11 = (byte) 0x54;
    public static final byte MPU9250_EXT_SENS_DATA_12 = (byte) 0x55;
    public static final byte MPU9250_EXT_SENS_DATA_13 = (byte) 0x56;
    public static final byte MPU9250_EXT_SENS_DATA_14 = (byte) 0x57;
    public static final byte MPU9250_EXT_SENS_DATA_15 = (byte) 0x58;
    public static final byte MPU9250_EXT_SENS_DATA_16 = (byte) 0x59;
    public static final byte MPU9250_EXT_SENS_DATA_17 = (byte) 0x5A;
    public static final byte MPU9250_EXT_SENS_DATA_18 = (byte) 0x5B;
    public static final byte MPU9250_EXT_SENS_DATA_19 = (byte) 0x5C;
    public static final byte MPU9250_EXT_SENS_DATA_20 = (byte) 0x5D;
    public static final byte MPU9250_EXT_SENS_DATA_21 = (byte) 0x5E;
    public static final byte MPU9250_EXT_SENS_DATA_22 = (byte) 0x5F;
    public static final byte MPU9250_EXT_SENS_DATA_23 = (byte) 0x60;
    public static final byte MPU9250_I2C_SLV0_DO = (byte) 0x63;
    public static final byte MPU9250_I2C_SLV1_DO = (byte) 0x64;
    public static final byte MPU9250_I2C_SLV2_DO = (byte) 0x65;
    public static final byte MPU9250_I2C_SLV3_DO = (byte) 0x66;
    public static final byte MPU9250_I2C_MST_DELAY_CTRL = (byte) 0x67;
    public static final byte MPU9250_SIGNAL_PATH_RESET = (byte) 0x68;
    public static final byte MPU9250_MOT_DETECT_CTRL = (byte) 0x69;
    public static final byte MPU9250_USER_CTRL = (byte) 0x6A;
    public static final byte MPU9250_PWR_MGMT = (byte) 0x6B;
    public static final byte MPU9250_PWR_MGMT_2 = (byte) 0x6C;
    public static final byte MPU9250_FIFO_COUNTH = (byte) 0x72;
    public static final byte MPU9250_FIFO_COUNTL = (byte) 0x73;
    public static final byte MPU9250_FIFO_R_W = (byte) 0x74;
    public static final byte MPU9250_WHO_AM_I = (byte) 0x75;
    public static final byte MPU9250_XA_OFFSET_H = (byte) 0x77;
    public static final byte MPU9250_XA_OFFSET_L = (byte) 0x78;
    public static final byte MPU9250_YA_OFFSET_H = (byte) 0x7A;
    public static final byte MPU9250_YA_OFFSET_L = (byte) 0x7B;
    public static final byte MPU9250_ZA_OFFSET_H = (byte) 0x7D;
    public static final byte MPU9250_ZA_OFFSET_L = (byte) 0x7E;

    // AK8963 Registers
    public static final byte AK8963_I2C_ADDR = (byte) 0x0C;   // slave address for the AK8963
    public static final byte AK8963_DEVICE_ID = (byte) 0x48;
    public static final byte AK8963_WIA = (byte) 0x00;
    public static final byte AK8963_INFO = (byte) 0x01;
    public static final byte AK8963_ST1 = (byte) 0x02;        // data ready status bit 0
    public static final byte AK8963_XOUT_L = (byte) 0x03;     // data
    public static final byte AK8963_XOUT_H = (byte) 0x04;
    public static final byte AK8963_YOUT_L = (byte) 0x05;
    public static final byte AK8963_YOUT_H = (byte) 0x06;
    public static final byte AK8963_ZOUT_L = (byte) 0x07;
    public static final byte AK8963_ZOUT_H = (byte) 0x08;
    public static final byte AK8963_ST2 = (byte) 0x09;        // Data overflow bit 3 and data read error status bit 2
    public static final byte AK8963_CNTL = (byte) 0x0A;       // Power down (0000), single-measurement (0001), 
                                                              // self-test (1000) and Fuse ROM (1111) modes on bits 3:0
    public static final byte AK8963_ASTC = (byte) 0x0C;       // Self test control
    public static final byte AK8963_TS1 = (byte) 0x0D;
    public static final byte AK8963_TS2 = (byte) 0x0E;
    public static final byte AK8963_I2CDIS = (byte) 0x0F;     // I2C disable
    public static final byte AK8963_ASAX = (byte) 0x10;       // Fuse ROM x-axis sensitivity adjustment value
    public static final byte AK8963_ASAY = (byte) 0x11;       // Fuse ROM y-axis sensitivity adjustment value
    public static final byte AK8963_ASAZ = (byte) 0x12;       // Fuse ROM z-axis sensitivity adjustment value

    // AK8963 Write/Read Registers
    public static final byte AK8963_CNTL1 = (byte) 0x0A;
    public static final byte AK8963_CNTL2 = (byte) 0x0B;
    
    /*
     *The unit of magnetic force reported here is the Tesla -  A particle, carrying a charge of one coulomb, 
     *and moving perpendicularly through a magnetic field of one tesla, at a speed of one metre per second, 
     *experiences a force with magnitude one newton, according to the Lorentz force law.
     * 
     * Data on the 3 axes of the AK8963 is stored as two's complement, 
     * Little Endian. 0x7ff8 or 32,760 = 4912 micro Tesla while 0x8008 or -32,760 = -4912 micro Tesla.
     * 
     * The unit Tesla is kilogram per second squared per amp.
     * 
     * Parkdale High School is located at 38.9694012 N and 76.9739942 W.  The magnetic flux of the earth's
     * poles there is 10.89 degrees west, currently changing by 0.01 degrees west each year.
     */
    		




    /**
     * Program Main Entry Point
     *
     * @param args
     * @throws InterruptedException
     * @throws PlatformAlreadyAssignedException
     * @throws IOException
     * @throws UnsupportedBusNumberException
     */
    public void init() throws InterruptedException, PlatformAlreadyAssignedException, IOException, UnsupportedBusNumberException {

        // create Pi4J console wrapper/helper
        // (This is a utility class to abstract some of the boilerplate code)
        final Console console = new Console();

        // print program title/header
        console.title("<-- The Pi4J Project -->", "I2C Example");

        // allow for user to exit program using CTRL-C
        console.promptForExit();

        // get the I2C bus to communicate on
        I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);

        // create an I2C device for an individual device on the bus that you want to communicate with
        // in this example we will use the default address for the MPU9250 chip which is 0x68.
        mpudevice = i2c.getDevice(MPU9250_ADDR);

        // next, lets perform am I2C READ operation to the MPU9250 chip
        // we will read the 'ID' register from the chip to get its part number and silicon revision number
        console.println("... reading ID register from MPU9250");
        int response = mpudevice.read(MPU9250_WHO_AM_I);
        console.println("MPU9250 ID = " + String.format("0x%02x", response) + " (should be 0x71)");

        // next we want to set device to initialize Magnetometer (AK8963)
        console.println("... starting up AK8963");
        mpudevice.write(MPU9250_INT_PIN_CFG, (byte) 0x22);

        // wait while the chip collects data
        Thread.sleep(500);
        
        // create an I2C device for an individual device on the bus that you want to communicate with
        // in this example we will use the default address for the AK6893 chip which is 0x0C.
        magdevice = i2c.getDevice(AK8963_ADDR);

    
    }
    
    /*
     * Set up some service routines down here once the magnetometer works. 
     * 
     */
    public int[] readMagnetometer()  throws InterruptedException, PlatformAlreadyAssignedException, IOException, UnsupportedBusNumberException {
    	int[] a = {1,2,3};

    	// Read and store X, Y, & Z Values from AK8693
        a[0] = magdevice.read( AK8963_XOUT_L ) + magdevice.read( AK8963_XOUT_H ) * 256;
        a[1] = magdevice.read( AK8963_YOUT_L ) + magdevice.read( AK8963_YOUT_H ) * 256;
        a[2] = magdevice.read( AK8963_ZOUT_L ) + magdevice.read( AK8963_ZOUT_H ) * 256;
 
    	return a;    	
    }

    public int[] readAccelerometer()  throws InterruptedException, PlatformAlreadyAssignedException, IOException, UnsupportedBusNumberException {
    	int[] a = {1,2,3};

       // Read and store X, Y, & Z Values from MPU9250 Accelerometer
        a[0] = mpudevice.read( MPU9250_ACCEL_XOUT_L ) + mpudevice.read( MPU9250_ACCEL_XOUT_H ) * 256;
        a[1] = mpudevice.read( MPU9250_ACCEL_YOUT_L ) + mpudevice.read( MPU9250_ACCEL_YOUT_H ) * 256;
        a[2] = mpudevice.read( MPU9250_ACCEL_ZOUT_L ) + mpudevice.read( MPU9250_ACCEL_ZOUT_H ) * 256;
 
    	return a;    	
    }

    public int[] readGyro()  throws InterruptedException, PlatformAlreadyAssignedException, IOException, UnsupportedBusNumberException {
    	int[] a = {1,2,3};

        // Read and store X, Y, & Z Values from MPU9250 Gyro
        a[0] = mpudevice.read( MPU9250_GYRO_XOUT_L ) + mpudevice.read( MPU9250_GYRO_XOUT_H ) * 256;
        a[1] = mpudevice.read( MPU9250_GYRO_YOUT_L ) + mpudevice.read( MPU9250_GYRO_YOUT_H ) * 256;
        a[2] = mpudevice.read( MPU9250_GYRO_ZOUT_L ) + mpudevice.read( MPU9250_GYRO_ZOUT_H ) * 256;
    
 
    	return a;    	
    }


}