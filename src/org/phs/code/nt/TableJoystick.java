package org.phs.code.nt;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class TableJoystick {
	private NetworkTable table;
	private double pov1;
	private double pov2;
	private double leftX;
	private double leftY;
	private double rightX;
	private double rightY;
	private double button01;
	private double button02;
	private double button03;
	private double button04;
	private double button05;
	private double button06;
	private double button07;
	private double button08;
	private double button09;
	private double button10;
	private double button11;
	private double button12;

  public TableJoystick() {
	  
	  NetworkTable.setIPAddress("10.49.49.191");
	  NetworkTable.setClientMode();
	  table = NetworkTable.getTable("tiny/joy");
	 
	  leftX = table.getNumber("LEFT X", 0);
	  leftY = table.getNumber("LEFT Y", 0);
	  rightX = table.getNumber("RIGHT X", 0);
	  rightY = table.getNumber("RIGHT Y", 0);
	  pov1 = table.getNumber("Coolie Cap X", 0);
	  pov2 = table.getNumber("Coolie Cap Y", 0);
	  button01 = table.getNumber("BUTTON 01", 0);
	  button02 = table.getNumber("BUTTON 02", 0);
	  button03 = table.getNumber("BUTTON 03", 0);
	  button04 = table.getNumber("BUTTON 04", 0);
	  button05 = table.getNumber("BUTTON 05", 0);
	  button06 = table.getNumber("BUTTON 06", 0);
	  button07 = table.getNumber("BUTTON 07", 0);
	  button08 = table.getNumber("BUTTON 08", 0);
	  button09 = table.getNumber("BUTTON 09", 0);
	  button10 = table.getNumber("BUTTON 10", 0);
	  button11 = table.getNumber("BUTTON 11", 0);
	  button12 = table.getNumber("BUTTON 12", 0);

  }

  public double[] PollMaster() {
	  double[] keyTable = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	  leftX = table.getNumber("LEFT X", 0);
	  leftY = table.getNumber("LEFT Y", 0);
	  rightX = table.getNumber("RIGHT X", 0);
	  rightY = table.getNumber("RIGHT Y", 0);
	  pov1 = table.getNumber("Coolie Cap X", 0);
	  pov2 = table.getNumber("Coolie Cap Y", 0);
	  button01 = table.getNumber("BUTTON 01", 0);
	  button02 = table.getNumber("BUTTON 02", 0);
	  button03 = table.getNumber("BUTTON 03", 0);
	  button04 = table.getNumber("BUTTON 04", 0);
	  button05 = table.getNumber("BUTTON 05", 0);
	  button06 = table.getNumber("BUTTON 06", 0);
	  button07 = table.getNumber("BUTTON 07", 0);
	  button08 = table.getNumber("BUTTON 08", 0);
	  button09 = table.getNumber("BUTTON 09", 0);
	  button10 = table.getNumber("BUTTON 10", 0);
	  button11 = table.getNumber("BUTTON 11", 0);
	  button12 = table.getNumber("BUTTON 12", 0);
	  keyTable[0] = leftX;
	  keyTable[1] = leftY;
	  keyTable[2] = rightX;
	  keyTable[3] = rightY;
	  keyTable[4] = pov1;
	  keyTable[5] = pov2;
	  keyTable[6] = button01;
	  keyTable[7] = button02;
	  keyTable[8] = button03;
	  keyTable[9] = button04;
	  keyTable[10] = button05;
	  keyTable[11] = button06;
	  keyTable[12] = button07;
	  keyTable[13] = button08;
	  keyTable[14] = button09;
	  keyTable[15] = button10;
	  keyTable[16] = button11;
	  keyTable[17] = button12;
	  return keyTable;
  }


  /*
   * These routines can be used to bring back each key value as last read 
   * in the shared tables.
   */
  public double getLeftX() {
	  return leftX;
  }

  public double getLeftY() {
	  return leftY;
  }
  
  public double getRightX() {
	  return  rightX;
  }
  public double getRightY() {
	  return  rightY;
  }
  public double getPOV1() {
	  return  pov1;
  }
  public double getPOV2() {
	  return  pov2;
  }
  public double getButton01() {
	  return  button01;
  }
  public double getButton02() {
	  return  button02;
  }
  public double getButton03() {
	  return  button03;
  }
  public double getButton04() {
	  return  button04;
  }
  public double getButton05() {
	  return  button05;
  }
  public double getButton06() {
	  return  button06;
  }
  public double getButton07() {
	  return  button07;
  }
  public double getButton08() {
	  return  button08;
  }
  public double getButton09() {
	  return  button09;
  }
  public double getButton10() {
	  return  button10;
  }
  public double getButton11() {
	  return  button11;
  }
  	public double getButton12() {
	  return button12;
  }

}