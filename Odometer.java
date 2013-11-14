
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class Navigator extends Thread {
	private static final int FORWARD_SPEED = 250; 
	private static final int ROTATE_SPEED = 150;
	private final static double LEFT_RADIUS = 2.12; //left wheel radius
	private final static double RIGHT_RADIUS = 2.12; //right wheel radius
	private final static double W = 14.2; //distance between wheels
	static NXTRegulatedMotor leftMotor = Motor.A;
	static NXTRegulatedMotor rightMotor = Motor.B; 
	
	private static boolean isTraveling; //checks if the robot is moving or not
	private static Odometer odometer; 
	private static final double bandwidth = 2.00; 
	private static final double thetaBandwidth = 0.04; //in radians
	private static double xError;
	private static double yError;
	private static double thetaError; 
	private static double xCur;
	private static double yCur; 
	
	private static UltrasonicSensor usSensor = new UltrasonicSensor(SensorPort.S2); //added 
			
	public Navigator(Odometer odometer) {
		this.odometer = odometer; 
	}

	//makes the robot travel to destination (x,y) 
	public static void travelTo(double x, double y) {
		xCur = x; //gets current values of x
		yCur = y; //gets current values of y 

		isTraveling = false; //robot is not moving

		turnTo(Math.atan2((yCur-odometer.getY()), (xCur-odometer.getX()))); //theta in rads
		
		isTraveling = true; //now robot is moving 
		while (isTraveling) {
			avoid(); //avoid method always running - if no obstacle, acts as normal
			xError = xCur - odometer.getX(); //desired x distance minus odometer distance
			yError = yCur - odometer.getY(); //desired y distance minus odometer distance
			
			//if error of both x,y is less than 2  cm, robot will stop
			if ((Math.abs(xError) <= bandwidth) && (Math.abs(yError) <= bandwidth)) {
				leftMotor.stop();
				rightMotor.stop();
				isTraveling = false;
				break; 
			}
		 turnTo(Math.atan2((yCur-odometer.getY()), (xCur-odometer.getX())));
		  }
	}
	
	public static void turnTo(double theta) {
		//theta in radians
		//what to do if robot isnt traveling - rotate to correct position
		thetaError = theta - odometer.getTheta(); //desired theta minus odometer theta

		//turns initially to desired heading
		if (!isTraveling) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			leftMotor.rotate(-convertAngle(LEFT_RADIUS, W, Math.toDegrees(thetaError)), true);
			rightMotor.rotate(convertAngle(RIGHT_RADIUS, W, Math.toDegrees(thetaError)), false);
			
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			
			leftMotor.forward();
			rightMotor.forward();
		}
		// what to do while traveling - p controller to correct angle heading
			else if (Math.abs(thetaError) >= thetaBandwidth) {
				if (thetaError > 0) {
				leftMotor.setSpeed(ROTATE_SPEED + (int) (10*thetaError));
				rightMotor.setSpeed(ROTATE_SPEED - (int) (10*thetaError));
				}
				else {
					leftMotor.setSpeed(ROTATE_SPEED - (int) (10*thetaError));
					rightMotor.setSpeed(ROTATE_SPEED + (int) (10*thetaError));
				}
			}
			//what to do if theta is okay - just continue to travel straight 
			else {
				leftMotor.forward();
				rightMotor.forward(); 
			}
		}

	//returns whether robot is traveling or not
	public static boolean isNavigating() {
		return isTraveling; 
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	//avoids obstacles
	private static void avoid() {
		if (usSensor.getDistance() <= 10) { //added
			
			leftMotor.stop();
			rightMotor.stop();
			isTraveling = false; 
			
			// turn 90 degrees counter clockwise 
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(LEFT_RADIUS, W, 90.0), true);
			rightMotor.rotate(convertAngle(RIGHT_RADIUS, W, 90.0), false);
			
			// drive forward
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(LEFT_RADIUS, 26), true);
			rightMotor.rotate(convertDistance(RIGHT_RADIUS, 26), false);
			leftMotor.stop();
			rightMotor.stop();
			
			// turn 90 degrees clockwise
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(LEFT_RADIUS, W, 90.0), true);
			rightMotor.rotate(-convertAngle(RIGHT_RADIUS, W, 90.0), false);
			
			// drive forward
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(LEFT_RADIUS, 37), true);
			rightMotor.rotate(convertDistance(RIGHT_RADIUS, 37), false);
			leftMotor.stop();
			rightMotor.stop();

			// turn 90 degrees clockwise
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(LEFT_RADIUS, W, 90.0), true);
			rightMotor.rotate(-convertAngle(RIGHT_RADIUS, W, 90.0), false);
			
			// drive forward
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(LEFT_RADIUS, 30), true);
			rightMotor.rotate(convertDistance(RIGHT_RADIUS, 30), false);
			leftMotor.stop();
			rightMotor.stop();
			
			//rotate by 90 degrees counter clockwise
			//robot back to initial trajectory
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(LEFT_RADIUS, W, 90.0), true);
			rightMotor.rotate(convertAngle(RIGHT_RADIUS, W, 90.0), false);
			
			isTraveling = true; 
			rightMotor.forward();
			leftMotor.forward();
		}
	}
}
