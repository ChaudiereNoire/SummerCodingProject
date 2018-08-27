package org.phs.code.robot;

import Jama.*;

/*
 * The state of our vehicle is determined by its position in the matrix and its angle.
 * These items, taken together, are considered a pose.
 * 
 * The position of a robot can be designated in three dimensions as r = {x, y, z}.  A 
 * fourth element, omega, often designated as w, can be used to show the orientation of 
 * the robot on the field in degrees from some arbitrary axis.
 * 
 * Translation moves the robot, in the same orientation, to a new position on the field.
 * 
 * Rotation reorients the robot around one body axis, leaving it in the same position.
 *  - Roll/bank (psi) takes place along the front to back axis of a vehicle
 *  - Pitch/attitude (phi) takes place along the right to left axis of a vehicle
 *  - Yaw/heading (theta) takes place around the up down axis 
 *  
 *   - x red
 *   - y green
 *   - z blue
 *  
 *  
 */


public class StateAssessment {
	private double[] pose = {0.0, 0.0, 0.0, 0.0};
	private double[] target = {0.0, 0.0, 0.0, 0.0};
	long newTime = 0;
	long prevTime = 0;
	
	double newLeftCount = 0;
	double prevLeftCount = 0;
	double newRightCount = 0;
	double prevRightCount = 0;
	
	
	double dArray [] = { 0, 0, 0};
	
	double ticks_per_centimeter = 19.5;
	double theta = 0;

	
	/*
	 * The initial state for this exercise is the robot sitting with its axle straddling 
	 * a point 1.5 meters left of center and 1.5 meters back from center, pointing directly
	 * perpendicular from the back of the field.  The goal is 1.5 meters to the right of 
	 * center and 1.5 meters forward from the center.  The center of the course is blocked
	 * so that the vehicle cannot actually cross directly to the target at 45 degrees. 
	 */	
	public void init() {
		System.out.println("Preparing initial state assessment.");
		pose[0] = -150.0;
		pose[1] = -150.0;
		pose[2] = 0.0;
		pose[3] = 0.0;
		
		target[0] = 150.0;
		target[1] = 150.0;
		target[2] = 0.0;
		target[3] = 0.0;
		
		newTime = System.currentTimeMillis();
		prevTime = System.currentTimeMillis();

	}
	
	/*
	 * This routine computes the current pose assuming that the 67 milliseconds we are using on each
	 * cycle of driving the direction is close enough to being a straight line that the estimate will 
	 * not be all that far off if we pretend that it is a straight line.  We use the Instantaneous 
	 * Center of Curvature ICC) of the chord we are dring on to work out the position and orientation 
	 * of the vehicle on the field.  Omega, the rate of rotation about the ICC, is, by definition, the 
	 * same for both wheels.
	 * 
	 * Notice that R, the distance to the ICC, is a signed number.  If the right wheel is moving faster than
	 * the left wheel, the vehicle is going counter clockwise around the ICC, if the left wheel is moving
	 * faster than the right wheel, it is going around the ICC in a clockwise direction.
	 * 
	 * The variable "l" is the width of the wheel base from center to center of the tires.  Ours is 15 cm.
	 * If "R" is the distance from the center of the axle to the ICC, then the velocity of the right wheel is 
	 * equal to omega times the quantity R + l/2 and the velocity of the left wheel is equal to omega times 
	 * the quantity R - l/2.
	 * 
	 * The value of R is equal to 1/2 the quantity that is computed by dividing the sum of the left and right 
	 * velocities by the difference between the velocity of the right wheel and the left wheel.  Omega is 
	 * equal to the difference between the velocity of the right wheel and the left wheel divided by l.
	 */
	public void computeCurrentPose( long currentTime, double leftCount, double rightCount) {

		double l = 15;
		Matrix A = new Matrix(3,3, 0.0);
		Matrix B = new Matrix(3,1, 0.0);
		Matrix C = new Matrix(3,1, 0.0);
		Matrix D = new Matrix(3,1, 0.0);

		prevLeftCount = newLeftCount;
		prevRightCount = newRightCount;

		newLeftCount = leftCount;
		newRightCount = rightCount;

		prevTime = newTime;
		newTime = currentTime;

		long dT = currentTime - prevTime;
		double delta_left = (newLeftCount - prevLeftCount) / ticks_per_centimeter;
		double delta_right = (newRightCount - prevRightCount) / ticks_per_centimeter;
		
		dArray[0] += (delta_left + delta_right)/2;
		dArray[1] += delta_left;
		dArray[2] += delta_right;

		/*
		 * Calculate the rotational velocity in radians per unit time
		 */
		double omega = 0;
		if (dT == 0) {
			omega = 0;
		} else {
			omega = (delta_right/dT - delta_left/dT) / l;
		}

		/*
		 * Find the value of R and compute its x and y coordinates
		 */
		theta = pose[3];
		if (dT < 1) {
			D.set(0, 0, pose[0]); 
			D.set(1, 0, pose[1]);
			D.set(2, 0, theta);
		} else {
			if (delta_right == delta_left) {
				D.set(0, 0, pose[0] + delta_right * Math.cos(theta)); 
				D.set(1, 0, pose[1] + delta_right * Math.sin(theta));
				D.set(2, 0, theta);

			} else {
				double R = 0.5 * (delta_right/dT + delta_left/dT)/(delta_right/dT - delta_left/dT);
				
				/*
				 * Rotation around Z axis
				 */
				A.set(0, 0, Math.cos(omega * dT));
				A.set(0, 1, -Math.sin(omega * dT));
				A.set(0, 2, 0.0);
				A.set(1, 0, Math.sin(omega * dT));
				A.set(1, 1, Math.cos(omega * dT));
				A.set(1, 2, 0.0);
				A.set(2, 0, 0.0);
				A.set(2, 1, 0.0);
				A.set(2, 2, 1.0);
				
				/*
				 * Translate ICC to Origin
				 */
				B.set(0, 0, pose[0] - (pose[0] - R * Math.sin(theta))); 
				B.set(1, 0, pose[1] - (pose[1] + R * Math.cos(theta)));
				B.set(2, 0, theta);

				/*
				 * Translate Origin back to ICC
				 */
				C.set(0, 0, pose[0] - R * Math.sin(theta)); 
				C.set(1, 0, pose[1] + R * Math.cos(theta));
				C.set(2, 0, omega * dT);
				
				D = A.times(B).plus(C);
				
			}
		}
		
		double newX = D.get(0,  0);
		double newY = D.get(1,  0);
		double newZ = 0.0;
		double newTheta = D.get(2,  0);

		setCurrentPose( newX, newY, newZ, newTheta);
		
	}

	
	
	/*
	 * The current pose is determined by working the vector last traveled by the vehicle and 
	 * adding the result to the last recorded state based on the amount of time passed. 
	 */	
	public double[] getCurrentPose() {
		return pose;
	}
	
	
	/*
	 * The distance traveled is just the sum of the distance traveled by each wheel.  In 
	 * addition, the distance covered by the center of the axle is reported. 
	 */	
	public double[] getDistanceTravelled() {
		return dArray;
	}
	
	/*
	 * The current pose is determined by working the vector last traveled by the vehicle and 
	 * adding the result to the last recorded state based on the amount of time passed. 
	 */	
	public double[] getCurrentTarget() {
		return target;
	}
	
	/*
	 * The current pose is determined by working the vector last traveled by the vehicle and 
	 * adding the result to the last recorded state based on the amount of time passed.
	 * If you are calculating these values outside StateAssessemtn, this is the method that you
	 * would call to set them. 
	 */	
	public void setCurrentPose(double x, double y, double z, double theta) {
		pose[0] = x;
		pose[1] = y;
		pose[2] = z;
		pose[3] = theta;
	}

	/*
	 * The vehicle is at the destination when subtracting the current pose x and y from the destination
	 *  x and y results in a circle less than 10 centimeters in diameter. 
	 */	
	public boolean atDestination() {
		double x = target[0] - pose[0];
		double y = target[1] - pose[1];
		double distance = Math.sqrt(x * x + y * y);
		return (distance < 15);
	}

}