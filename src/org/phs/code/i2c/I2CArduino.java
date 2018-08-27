package org.phs.code.i2c;

import java.io.IOException;
import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.io.i2c.I2CFactory.UnsupportedBusNumberException;
import com.pi4j.platform.PlatformAlreadyAssignedException;

/**
*  The Arduino is expected to be found at I2C address 0x04 and running
*  a variation of the sample code that was published on ScreenStepsLive.
*  
*  http://wpilib.screenstepslive.com/s/3120/m/7912/l/175524-sending-data-from-the-crio-to-an-arduino
*  
*  The orginal example is published below as the method blinkLED().  Note that the Arduino 
*  has the address as 84, but the wpilib as 168 - these are the same address.
*  
*  //i2c Slave Code (Uno)
*  #include <Wire.h>
*  
*  void setup() {
*      Wire.begin(84);
*      Wire.onReceive(receiveEvent);
*      
*      pinMode(13,OUTPUT);
*      digitalWrite(13,LOW);
*  }
*  
*  void loop() {
*  }
*  
*  void receiveEvent(int howMany) {
*      while(Wire.available()) {
*          char c = Wire.read();
*          if(c == 72) digitalWrite(13,HIGH);
*          else if(c == 76) digitalWrite(13,LOW);
*       }
*   }
*   
*   A more complete implementation for the Arduino is found at MDHS Robotics Mater Dei High School Team 4141
*   
*   https://github.com/MDHSRobotics/I2CArduino
*   
*/
public class I2CArduino {
	
	private static final int ARDUINO_ADDR = 04;

	public I2CDevice arduino;

 boolean on = false;
	byte[] data;
	int address;
	int value1 = 0;
	int value2 = 0;
	int value3 = 0;
	byte[] toSend = new byte[1];
	byte[] toGet = new byte[1];
		
	byte[] recvData = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			           0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


 public void init() throws InterruptedException, PlatformAlreadyAssignedException, IOException, UnsupportedBusNumberException  {
     // get the I2C bus to communicate on
     I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
     
     // create an I2C device for an individual device on the bus that you want to communicate with
     // in this example we will use the default address for the MPU9250 chip which is 0x68.
     arduino = i2c.getDevice(ARDUINO_ADDR);


 }
 /*
  * The Write function feeds three bytes of data to the RioDuino for command
  * instructions.  They were used to set the parameters for the digital rangefinders
  * the code was originally written to support.
  */
 		
 	public void Write(int val1, int val2, int val3){
 		data[0] = (byte) val1;
 		data[1] = (byte) val2;
 		data[2] = (byte) val3;
 		//i2c.writeBulk(data);
 	}
 	
 /*
  * The readRegister function really does not require an argument.  It was originally
  * envisioned as returning the values for one register. On reflections, it seemed that,
  * for demonstration purposes, I might look at returning the entire register set in one
  * pass and then process that all for one cycle.  It becomes an interesting engineering
  * problem to determine if this is a good strategy or not.	
  */
 	public void readRegister(int register) {
 		for (byte i=0; i < 33; i++) {
 			toSend[0] = i;
 			toGet[0] = 111;
 			//i2c.transaction(toSend, 1, toGet,1);
 			recvData[i] = toGet[0];
 			System.out.println("Register " + i + "  Requested.  Byte Received: " + toGet[0]);
 			//Timer.delay(.01);
 		}
 	}
 	
 /*
  * This routine was borrowed from team 1403 and generalized.  It depends on the
  * RioDuino returning values byte by byte after converting the integer to byte values.
  * The coding is reminiscent of EBCDIC.
  * 
  * In the routine that reads the Arduino registers, the data is stored as an exact copy
  * of the Arduino register map, with four bytes per analog value.  This routing convers
  * the values back to the value that came from the analog registers.
  * 
  *   byte1 = int((value/1000));
  *   byte2 = int((value/100)%10);
  *   byte3 = int((value/10)%10);
  *   byte4 = int((((value%1000)%100)%10));
  *	
  */
 	public int getAnalogRegister(int register){
 		int value,byte1,byte2,byte3,byte4,lookupReg;
 		boolean isNegative=false;
 		lookupReg = register * 4;
 		byte1 = recvData[lookupReg+4];
 		byte2 = recvData[lookupReg+5];
 		byte3 = recvData[lookupReg+6];
 		byte4 = recvData[lookupReg+7];

 		if(Math.signum(byte1)==-1||
 		   Math.signum(byte2)==-1||
 		   Math.signum(byte3)==-1||
 		   Math.signum(byte3)==-1){
 			byte1*=-1;
 			byte2*=-1;
 			byte3*=-1;
 			byte4*=-1;
 			isNegative=true;
 		}
 		String bytestring = ""+byte1+byte2+byte3+byte4;
 		System.out.println(bytestring);
 		value =Integer.parseInt(bytestring);
 		if(isNegative){
 			value*=-1;
 		}
 		return value;
 	}
 	
 /*
  * This routine follows the same method used for the analog registers, but the values
  * of the digital signals were packed into one set of integers before being stored in
  * the array. 
  * 
  *   pinVal = digitalRead(1) * 1;
  *   pinVal += digitalRead(2) * 2;
  *   pinVal += digitalRead(3) * 8;
  *   pinVal += digitalRead(4) * 16;
  *   pinVal += digitalRead(5) * 32;
  *   pinVal += digitalRead(6) * 64;
  *   pinVal += digitalRead(7) * 128;
  *   pinVal += digitalRead(8) * 256;
  *   pinVal += digitalRead(9) * 512;
  *   pinVal += digitalRead(10) * 1024;
  *   pinVal += digitalRead(11) * 2048;
  *   pinVal += digitalRead(12) * 4096;
  *   pinVal += digitalRead(13) * 8192;
  *   value = pinVal;
  *   regMap[28] = byte((value/1000));
  *   regMap[29] = byte((value/100)%10);
  *   regMap[30] = byte((value/10)%10);
  *   regMap[31] = byte((((value%1000)%100)%10));
  *
  */
 	
 	public int getDigitalRegister(int register){
 		int value,byte1,byte2,byte3,byte4,lookupReg;
 		boolean isNegative=false;
 	    lookupReg = 28;
 		byte1 = recvData[lookupReg];
 		byte2 = recvData[lookupReg+1];
 		byte3 = recvData[lookupReg+2];
 		byte4 = recvData[lookupReg+3];

 		if(Math.signum(byte1)==-1||
 		   Math.signum(byte2)==-1||
 		   Math.signum(byte3)==-1||
 		   Math.signum(byte3)==-1){
 			byte1*=-1;
 			byte2*=-1;
 			byte3*=-1;
 			byte4*=-1;
 			isNegative=true;
 		}
 		String bytestring = ""+byte1+byte2+byte3+byte4;
 		System.out.println(bytestring);
 		value = Integer.parseInt(bytestring);
 		if(isNegative){
 			value*=-1;
 		}
 		
 		String s = new String("0000000000000000" + Integer.toBinaryString(value));
 		int endIndex = s.length() - register;
 		int beginIndex = endIndex - 1;
 	    String retVal = (s.substring(beginIndex, endIndex) );
 		char a = retVal.charAt(0);
 		return a - 48;
 	}
}

