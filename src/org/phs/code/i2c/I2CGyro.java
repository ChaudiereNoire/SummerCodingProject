package org.phs.code.i2c;

/* Register offsets for the L3G4200D taken from 
 * https://www.elecrow.com/download/L3G4200_AN3393.pdf
 * WHO_AM_I  0x0f
 * CTRL_REG1 0x20
 *  BIT7 DR1
 *  BIT6 DR0
 *  BIT5 BW1
 *  BIT4 BW0 
 *  BIT3 PD
 *  BIT2 Zen
 *  BIT1 Yen
 *  BIT0 Xen
 * CTRL_REG2 0x21
 *  BIT7 0
 *  BIT6 0
 *  BIT5 HPM1
 *  BIT4 HPM0
 *  BIT3 HPCF3
 *  BIT2 HPCF2
 *  BIT1 HPCF1
 *  BIT0 HPCF0
 * CTRL_REG3 0x22
 *  BIT7 I1_Int1
 *  BIT6 I1_Boot
 *  BIT5 H_Lactive
 *  BIT4 PP_OD
 *  BIT3 I2_DRDY
 *  BIT2 I2_WTM
 *  BIT1 I2_ORun
 *  BIT0 I2_Empty
 * CTRL_REG4 0x23
 *  BIT7 BDU
 *  BIT6 BLE
 *  BIT5 FS1
 *  BIT4 FS0 
 *  BIT3 -
 *  BIT2 ST1
 *  BIT1 ST0
 *  BIT0 SIM
 * CTRL_REG5 0x24
 *  BIT7 BOOT
 *  BIT6 FIFO_EN
 *  BIT5 --
 *  BIT4 HPen 
 *  BIT3 INT1_Sel1
 *  BIT2 INT1_Sel0
 *  BIT1 OUT_Sel1
 *  BIT0 OUT_Sel0
 * REFERENCE 0x25
 * OUT_TEMP 0x26
 * STATUS_REG 0x27
 *  BIT7 ZYXOR
 *  BIT6 ZOR
 *  BIT5 YOR
 *  BIT4 XOR 
 *  BIT3 ZYXDA
 *  BIT2 ZDA
 *  BIT1 YDA
 *  BIT0 XDA
 * OUT_X_L 0x28
 * OUT_X-H 0x29
 * OUT_Y_L 0x2a
 * OUT_Y_H 0x2b
 * OUT_Z_L 0x2c
 * OUT_Z_H 0x2d
 * FIFO_CTRL_REG 0x2e
 *  BIT7 FM2
 *  BIT6 FM1
 *  BIT5 FM0
 *  BIT4 WTM4 
 *  BIT3 WTM3
 *  BIT2 WTM2
 *  BIT1 WTM1
 *  BIT0 WTM0
 * FIFO_SRC_REG 0x2f
 *  BIT7 WTM
 *  BIT6 OVRN
 *  BIT5 EMPTY
 *  BIT4 FSS4
 *  BIT3 FSS3
 *  BIT2 FSS2
 *  BIT1 FSS1
 *  BIT0 FSS0
 * INT1_CFG 0x30
 * INT1_SRC 0x31
 * INT1_TSH_XH 0x32
 * INT1_TSH_XL 0x33
 * INT1_TSH_YH 0x34
 * INT1_TSH_YL 0x35
 * INT1_TSH_ZH 0x36
 * INT1_TSH_ZL 0x37
 * INT1_DURATION 0x38
 */

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.io.i2c.I2CFactory.UnsupportedBusNumberException;

import java.io.IOException;

import org.phs.code.gyros.Gyro;

public class I2CGyro extends Gyro {
	private int xGyro;
	private int yGyro;
	private int zGyro;
	private double conversionFactor;
	private double xOffset;
	private double yOffset;
	private double zOffset;
	private double xValue;
	private double yValue;
	private double zValue;
	private double xPrev;
	private double yPrev;
	private double zPrev;
	private double fxG;
	private double fyG;
	private double fzG;
	private double alpha = 0.9;
	
	private I2CBus bus;
	private I2CDevice device;
	
	public I2CGyro() throws UnsupportedBusNumberException, IOException {
		// Create I2C bus
		bus = I2CFactory.getInstance(I2CBus.BUS_1);
		
		// Get I2C device, MPU-9250 I2C address is 0x69(105)
		device = bus.getDevice(0x69);
		
		enable();
		setRange("Max");
		
		// Read the first buffer
		ReadData();

	}
	
	public void ReadData() throws IOException {
		// Read 6 bytes of data from address 0x28(40)
		// X lsb, X msb, Y lsb, Y msb, Z lsb, Z msb
		byte[] data = new byte[6];
		data[0] = (byte)device.read(0x28);
		data[1] = (byte)device.read(0x29);
		data[2] = (byte)device.read(0x2A);
		data[3] = (byte)device.read(0x2B);
		data[4] = (byte)device.read(0x2C);
		data[5] = (byte)device.read(0x2D);

		// Convert the values
		xGyro = ((data[1] & 0xFF) * 256) + (data[0] & 0xFF);
		if (xGyro > 32767) {
			xGyro -= 65536;
		}
		xOffset = getX();
		xPrev = 0;
		
		yGyro = ((data[3] & 0xFF) * 256) + (data[2] & 0xFF);
		if (yGyro > 32767) {
			yGyro -= 65536;
		}
		yOffset = getY();
		yPrev = 0;
		
		zGyro = ((data[5] & 0xFF) * 256) + (data[4] & 0xFF);
		if (zGyro > 32767) {
			zGyro -= 65536;
		}
		zOffset = getZ();
		zPrev = 0;

	}
	
	@Override
	public void enable() {
		// Enable X, Y, Z-Axis and disable Power down mode
		try {
			device.write(0x20, (byte)0x0F);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	@Override
	public void disable() {
		
	}
	
	public void setRange(String rangeString) {
		/*
		 * The frequency range of the L3G4200D is either 250, 500, 2000, though some
		 * people have the max rate as 2500 degrees per second, the manual shows 2000.
		 * 
		 *  FS1 FS0 = Rate
		 *   0   0  =  250 mdps
		 *   0   1  =  500 mdps
		 *   1   0  = 2000 mdps
		 *   1   1  = 2000 mdps
		 *   
		 *   OR the result with 0x80 to enable BDU and disable BLE
		 * 
		 * The conversionFactor multiplied by the reconstructed register value
		 * gives the rate in degrees per second for that reading at the current sampling
		 * rate.
		 */
		byte range = 0x00;
		if (rangeString == "Low") {
			conversionFactor = 0.00875;
			range = (byte) 0x80;
		}
		if (rangeString == "Medium") {
			conversionFactor = 0.0175;
			range = (byte) 0x90;
		}
		if (rangeString == "High") {
			conversionFactor = 0.07;
			range = (byte) 0xa0;
		}
		if (rangeString == "Max") {
			conversionFactor = 0.07;
			range = (byte) 0xb0;
		}
		try {
			device.write(0x23, range);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
	}
	
	@Override
	public double getX()  {
		/*
		 * This computation depends on the rate setting 
		 */
		fxG = xGyro * conversionFactor + xOffset;
		xValue = fxG * alpha + xPrev * ( 1 - alpha);
		xPrev = xValue;
		return xValue;
	}
	@Override
	public double getY()  {
		fyG = yGyro * conversionFactor + yOffset;
		yValue = fyG * alpha + yPrev * ( 1 - alpha);
		yPrev = yValue;
		return yValue;
	}
	
	@Override
	public double getZ()  {
		fzG = zGyro * conversionFactor + zOffset;
		zValue = fzG * alpha + zPrev * ( 1 - alpha);
		zPrev = zValue;
		return zValue;
	}
			
}