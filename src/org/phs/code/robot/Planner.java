package org.phs.code.robot;

public class Planner {
	long newTime = 0;
	long prevTime = 0;
	
	double newLeftCount = 0;
	double prevLeftCount = 0;
	double newRightCount = 0;
	double prevRightCount = 0;
	double ticks_per_centimeter = 10;
	double theta = 0;
	
	double setPointA = 0;
	double setPointB = 0;
	double desiredAngle = 0;
	
	double dT = 0;
	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	
	double kP = 0;
	double kI = 0;
	double kD = 0;
	
	double output = 0;
	
	
	/*
	 * Trial grid for a field representation.  Each number in the array represents the contents 
	 * of that 10 cm square.  0 is nothing, 1 is the vehicle, 5 is the target, 7 is a safe 
	 * waypoint, and 9 is an obstacle.  They also represent status or command information, with 1
	 * meaning "I have already been here", 7 meaning "Go on to the next cell with a 7 or a 5", and
	 * 5 meaning "Stop here."  We could use the other values for other commands.
	 * 
	 *  The elements in the top row of the grid are (0,0), (0,1), 0,2) ..... (0,38), (0,39), (0,40)
	 *  while the elements on the far left in each row are (0,0), (1,0), (2,0) ..... (38,0), 39,0), 
	 *  and (40,0).
	 *  
	 *  That means, for our purposes, that if you use this representation, you have to picture yourself
	 *  standing on the left side of the matrix looking toward the right.  The corner to your left is 
	 *  (0,0), the one to your right is (40,0), the corner on the far side to the left is (0,40), and the 
	 *  corner on the far side to the right is (40, 40).  Using  FTC view of the field, the corner to your 
	 *  left is (-20, -20), the one to your right is (20,-20), the corner on the far side to the left is 
	 *  (-20,20), and the corner on the far side to the right is (20, 20).  We can thus change between these
	 *  two views of the field by adding 20 to each of our FTC index values or subtracting 20 from each
	 *  of the field index values. 
	 */
	
	int[][] field = { 
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
	

	public void init() {
		// Set up field coordinates and obstacle location estimates
		field[19][19] = 9;
		field[19][20] = 9;
		field[19][21] = 9;
		field[20][19] = 9;
		field[20][20] = 9;
		field[20][21] = 9;
		field[21][19] = 9;
		field[21][20] = 9;
		field[21][21] = 9;

	}
	
	public double[] getCurrentWayPost(int motorSpeeds[] ) {
		/*
		 *  Iterate table to choose next intermediate goal.  They are at (16,16), (23,16), 
		 *  (24,17), (24,25), and the target itself (35,35).  The strategy in this model 
		 *  would be to check the cell in the direction of the current desired Angle and 
		 *  see if it is a 7 or a 5.  If it is neither, the path is at an end and the new 
		 *  wayPoint is the cell you are testing, otherwise, you step into that cell and
		 *  check again.
		 */
		double direction = motorSpeeds[4];
		int x_origin = (int) (20 + (motorSpeeds[2]/10));
		int y_origin = (int) (20 + (motorSpeeds[3]/10));
		int x = x_origin;
		int y = y_origin;
		x_origin += (int) (Math.sin(Math.toRadians(direction))+0.5);  
		y_origin += (int) (Math.cos(Math.toRadians(direction))+0.5);
		while (field[x_origin][y_origin] == 7) {
			x = x_origin;
			y = y_origin;
			x_origin +=  (int) (Math.sin(Math.toRadians(direction))+0.5);  
			y_origin +=  (int) (Math.cos(Math.toRadians(direction))+0.5);
		}
		
		double[] newWayPost = { 0, 0, 0};
		newWayPost[0] = x * 10 - 200;
		newWayPost[1] = y * 10 - 200;
		return newWayPost;
	}
	
	
	/*
	 * The next move is calculated by taking a direct sighting on the target and then 
	 * stepping through every square that would be passed through from the current pose
	 * to to the target.  If there is an object on the path, the course is pushed to the 
	 * right and down, staying just one square away from the obstacle on the left hand
	 *  until the object is bypassed.
	 *  
	 *  While we are working on that calculation, we are just going to set up an array of
	 *  waypoints that do provide a clear path and have the vehicle drive to them in turn. 
	 */
	public int[] calculateNextMove(double[] pose, double[] target) {
		
		int[] newTarget = {0, 0, 0, 0, 0};

		/* This section is where we are building the path planner.  It will, if it is ever
		 * finished, look over the entire field and choose the optimum path.  In the meantime,
		 * we have plotted out an OK path and marked each step on the path with the number 7.
		 */
		double xd = ((target[0] - pose[0]) / 10);
		double yd = ((target[1] - pose[1]) / 10);
		/*
		 * Make sure the potential path can fit into the field.  We should do something here other
		 * than arbitrarily chop off the excess path size.  Right now we are just going to keep
		 * to the bounds of our arrays.
		 */
		xd = Math.min(xd, 40);
		xd = Math.max(xd, 0);
		yd = Math.min(yd, 40);
		yd = Math.max(yd, 0);

		int x_origin = (int) (20 + (pose[0]/10));  
		int y_origin = (int) (20 + (pose[1]/10));
		/*
		 * Make sure that the vehicle stays within the field
		 */
		x_origin = Math.min(x_origin, 40);
		x_origin = Math.max(x_origin, 0);
		y_origin = Math.min(y_origin, 40);
		y_origin = Math.max(y_origin, 0);
		
		/* Test the 8 edges of the quadrant and see which one is the path.  It will contain
		 * the number 7.  If you don't find the path, you are lost and need to come up with 
		 * a rule to choose the next square.  If you are lost, you should probably just aim
		 * for the target and keep checking the square ahead to see if there is supposed to 
		 * be an obstacle there.  We will need a distance sensor to verify that condition.
		 * 
		 * Angles are in radians.
		 * 
		 * When I originally conceived of this scheme, I pictured 0 as straight along the y axis.
		 * This is wrong - 0 is parallel to the x axis in the inverse kinetcs problems.  There is
		 * no logical mapping to the real globe.  What we need is an orientation routine that 
		 * squares the reference grid with the globe and our onboard compass.
		 * 
		 * The lines of force near my home are slanted 10 degrees 59 minutes west.  Parkdale High
		 * School measures 10 degrees 56 minutes west.
		 */

		if (field[x_origin - 1][y_origin - 1] == 5) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 225;
		} else 		if (field[x_origin - 1][y_origin] == 5) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin) * 10 - 200;
			newTarget[4] = 270;
		} else 		if (field[x_origin - 1][y_origin + 1] == 5) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 315;
		} else 		if (field[x_origin][y_origin + 1] == 5) {
			newTarget[2] = (x_origin) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 0;
		} else 		if (field[x_origin + 1][y_origin + 1] == 5) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 45;
		} else 		if (field[x_origin + 1][y_origin] == 5) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin) * 10 - 200;
			newTarget[4] = 90;
		} else 		if (field[x_origin + 1][y_origin - 1] == 5) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 135;
		} else 		if (field[x_origin][y_origin - 1] == 5) {
			newTarget[2] = (x_origin) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 180;
		}
		
		if (field[x_origin - 1][y_origin - 1] == 7) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 225;
		} else 		if (field[x_origin - 1][y_origin] == 7) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin) * 10 - 200;
			newTarget[4] = 270;
		} else 		if (field[x_origin - 1][y_origin + 1] == 7) {
			newTarget[2] = (x_origin - 1) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 315;
		} else 		if (field[x_origin][y_origin + 1] == 7) {
			newTarget[2] = (x_origin) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 0;
		} else 		if (field[x_origin + 1][y_origin + 1] == 7) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin + 1) * 10 - 200;
			newTarget[4] = 45;
		} else 		if (field[x_origin + 1][y_origin] == 7) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin) * 10 - 200;
			newTarget[4] = 90;
		} else 		if (field[x_origin + 1][y_origin - 1] == 7) {
			newTarget[2] = (x_origin + 1) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 135;
		} else 		if (field[x_origin][y_origin - 1] == 7) {
			newTarget[2] = (x_origin) * 10 - 200;
			newTarget[3] = (y_origin - 1) * 10 - 200;
			newTarget[4] = 180;
		}
		
		/*
		 * Adjust the map bearing to the local grid by subtracting 90 degrees from the compass bearing and 
		 * storing it to the steering angle. 
		 */
		setSteeringAngle( newTarget[4] );
		field[x_origin][y_origin] = 1;

		/*
		 * We now have a new angle in desiredAngle and the old angle in pose[3].  Each cardinal point 
		 * going around the vehicle represents 45 degrees of turn.   The circumference of the wheel is
		 * 20 cm and the wheel base is 16 cm, which is a turning circle of 50 cm.  With the two motors
		 * set to plus and minus for the same speed one wheel rotation plus a smidge will turn the vehicle 
		 * 180 degrees.
		 */
		
		if (desiredAngle == pose[3]) {
			newTarget[0] = 100;
			newTarget[1] = 100;
		} else if (desiredAngle < pose[3]) {
			newTarget[0] = -100;
			newTarget[1] = 100;
		} else {
			newTarget[0] = 100;
			newTarget[1] = -100;
		}
			
		//Logging.consoleLog("Checking adjoining square x=%d, y=%d content:%d %d %d %d %d %d %d %d. Target %d, %d, %d, %d, %d",
		//		x_origin-1, y_origin-1, field[x_origin-1][y_origin-1], field[x_origin-1][y_origin], 
		//		field[x_origin-1][y_origin+1], field[x_origin][y_origin+1], field[x_origin+1][y_origin+1], 
		//		field[x_origin+1][y_origin], field[x_origin-1][y_origin-1], field[x_origin][y_origin-1],
		//		newTarget[0], newTarget[1], newTarget[2], newTarget[3], newTarget[4]);

		return newTarget;
		
	}
	
	/*
	 * The map axis of our steering grid is rotated 90 degrees from the reference used for steering.
	 * The grid is intended for human consumption, so we will leave it as is and simply subtract
	 * 90 degrees here.
	 */
	public void setSteeringAngle( double angle ) {
		desiredAngle = Math.toRadians( angle - 90 );
		
	}
	/*
	 * The map axis of our steering grid is rotated 90 degrees from the reference used for steering.
	 * The grid is intended for human consumption, so we will leave it as is and simply subtract
	 * 90 degrees here.
	 */
	public double getMapSteeringAngle( ) {
		return Math.toDegrees(desiredAngle) + 90;
		
	}

}
