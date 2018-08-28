package org.phs.code;

/*
 * ---------------------------------
 * *Generic   USB  Joystick
 * Type: Stick
 * Component Count: 18
 * Component 0: Z Rotation
 *     Identifier: rz
 *     ComponentType: Absolute Analog
 * Component 1: Z Axis
 *     Identifier: z
 *     ComponentType: Absolute Analog
 * Component 2: Z Axis
 *     Identifier: z
 *     ComponentType: Absolute Analog
 * Component 3: Y Axis
 *     Identifier: y
 *     ComponentType: Absolute Analog
 * Component 4: X Axis
 *     Identifier: x
 *     ComponentType: Absolute Analog
 * Component 5: Hat Switch
 *     Identifier: pov
 *     ComponentType: Absolute Digital
 * Component 6: Button 0
 *     Identifier: 0
 *     ComponentType: Absolute Digital
 * Component 7: Button 1
 *     Identifier: 1
 *     ComponentType: Absolute Digital
 * Component 8: Button 2
 *     Identifier: 2
 *     ComponentType: Absolute Digital
 * Component 9: Button 3
 *     Identifier: 3
 *     ComponentType: Absolute Digital
 * Component 10: Button 4
 *     Identifier: 4
 *     ComponentType: Absolute Digital
 * Component 11: Button 5
 *     Identifier: 5
 *     ComponentType: Absolute Digital
 * Component 12: Button 6
 *     Identifier: 6
 *     ComponentType: Absolute Digital
 * Component 13: Button 7
 *     Identifier: 7
 *     ComponentType: Absolute Digital
 * Component 14: Button 8
 *     Identifier: 8
 *     ComponentType: Absolute Digital
 * Component 15: Button 9
 *     Identifier: 9
 *     ComponentType: Absolute Digital
 * Component 16: Button 10
 *     Identifier: 10
 *     ComponentType: Absolute Digital
 * Component 17: Button 11
 *     Identifier: 11
 *     ComponentType: Absolute Digital
 * ---------------------------------
 */


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.java.games.input.*;


public class DriverStation {
	
	String joystick = "USB Joystick";
	Controller j1;

	public static void main(String[] args) {
		new DriverStation().run();
	}

	  public void run() {
		  NetworkTableEntry pov1;
		  NetworkTableEntry pov2;
		  NetworkTableEntry pov3;
		  NetworkTableEntry pov4;
		  NetworkTableEntry leftX;
		  NetworkTableEntry leftY;
		  NetworkTableEntry rightX;
		  NetworkTableEntry rightY;
		  NetworkTableEntry button01;
		  NetworkTableEntry button02;
		  NetworkTableEntry button03;
		  NetworkTableEntry button04;
		  NetworkTableEntry button05;
		  NetworkTableEntry button06;
		  NetworkTableEntry button07;
		  NetworkTableEntry button08;
		  NetworkTableEntry button09;
		  NetworkTableEntry button10;
		  NetworkTableEntry button11;
		  NetworkTableEntry button12;

		  NetworkTableInstance inst = NetworkTableInstance.create();
		  NetworkTable table = inst.getTable("tiny/joy");
		  inst.startServer();

		  leftX = table.getEntry("LEFT X");
		  leftY = table.getEntry("LEFT Y");
		  rightX = table.getEntry("RIGHT X");
		  rightY = table.getEntry("RIGHT Y");
		  pov1 = table.getEntry("Coolie Cap X");
		  pov2 = table.getEntry("Coolie Cap Y");
		  button01 = table.getEntry("BUTTON 01");
		  button02 = table.getEntry("BUTTON 02");
		  button03 = table.getEntry("BUTTON 03");
		  button04 = table.getEntry("BUTTON 04");
		  button05 = table.getEntry("BUTTON 05");
		  button06 = table.getEntry("BUTTON 06");
		  button07 = table.getEntry("BUTTON 07");
		  button08 = table.getEntry("BUTTON 08");
		  button09 = table.getEntry("BUTTON 09");
		  button10 = table.getEntry("BUTTON 10");
		  button11 = table.getEntry("BUTTON 11");
		  button12 = table.getEntry("BUTTON 12");

		  
		  Controller[] ca = ControllerEnvironment.getDefaultEnvironment().getControllers();
		  
		  for(int i =0;i<ca.length;i++){

	            /* Get the name of the controller */
	            if (ca[i].getType() == Controller.Type.STICK) {
	            	j1 = ca[i];
	            }
	        }
		  
		  while (j1.poll()) {
	         Component[] components = j1.getComponents();
	         
	         rightX.setDouble(components[1].getPollData());
	         rightY.setDouble(components[0].getPollData());
	         leftY.setDouble(components[3].getPollData());
	         leftX.setDouble(components[2].getPollData());
	         
	         pov1.setDouble(components[4].getPollData());
	         pov2.setDouble(components[5].getPollData());

	         button01.setDouble(components[6].getPollData());
	         button02.setDouble(components[7].getPollData());
	         button03.setDouble(components[8].getPollData());
	         button04.setDouble(components[9].getPollData());
	         button05.setDouble(components[10].getPollData());
	         button06.setDouble(components[11].getPollData());
	         button07.setDouble(components[12].getPollData());
	         button08.setDouble(components[13].getPollData());
	         button09.setDouble(components[14].getPollData());
	         button10.setDouble(components[15].getPollData());
	         button11.setDouble(components[16].getPollData());
	         button12.setDouble(components[17].getPollData());
	          
	          try {
	             Thread.sleep(50);
	          } catch (InterruptedException e) {
	             // TODO Auto-generated catch block
	             e.printStackTrace();
	          }
		  }
		  inst.stopServer();
	  }
}



