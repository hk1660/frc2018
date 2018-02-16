package org.usfirst.frc.team1660.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

	// Xbox controller button mappings
	public final static int A_BUTTON = 1;
	public final static int B_BUTTON = 2;
	public final static int X_BUTTON = 3;
	public final static int Y_BUTTON = 4;
	public final static int LB_BUTTON = 5;
	public final static int RB_BUTTON = 6;
	public final static int BACK_BUTTON = 7;
	public final static int START_BUTTON = 8;
	public final static int LEFT_JOY_BUTTON = 9;
	public final static int RIGHT_JOY_BUTTON = 10;
	public final static int LEFT_X_AXIS = 0;
	public final static int LEFT_Y_AXIS = 1;
	public final static int LT_AXIS = 2;
	public final static int RT_AXIS = 3;
	public final static int RIGHT_X_AXIS = 4;
	public final static int RIGHT_Y_AXIS = 5;
	public final static int POV_UP = 0;
	public final static int POV_UP_RIGHT = 45;
	public final static int POV_RIGHT = 90;
	public final static int POV_DOWN_RIGHT = 135;
	public final static int POV_DOWN = 180;
	public final static int POV_DOWN_LEFT = 225;
	public final static int POV_LEFT = 270;
	public final static int POV_UP_LEFT = 315;

	
	// Joystick ports
	public final static int DRIVER_JOYSTICK_PORT = 0;
	public final static int MANIPULATOR_JOYSTICK_PORT = 1;

	//Drive Joystick
	public final static int FORWARD_AXIS = RIGHT_Y_AXIS;
	public final static int STRAFE_AXIS = RIGHT_X_AXIS;
	public final static int TURN_AXIS = LEFT_X_AXIS;
	public final static int MOVING_Y_AXIS = RIGHT_Y_AXIS;
	public final static int MOVING_X_AXIS = RIGHT_X_AXIS;
	public final static int HEADING_Y_AXIS = LEFT_Y_AXIS;
	public final static int HEADING_X_AXIS = LEFT_X_AXIS;
	public final static int FIELD_DRIVING_FLAG_ON_BUTTON = RB_BUTTON;
	public final static int FIELD_DRIVING_FLAG_OFF_BUTTON = LB_BUTTON;
	public final static int RESET_OFFSET_ANGLE_BUTTON = START_BUTTON;
	public final static int FACE_BACKWARD_POV = POV_DOWN;
	public final static int FACE_LEFT_POV = POV_LEFT;
	public final static int FACE_RIGHT_POV = POV_RIGHT;
	public final static int FACE_FORWARD_POV = POV_UP;
	
	
	//Manip Joystick
	public final static int LIFT_AXIS = LEFT_Y_AXIS;	
	public final static int EAT_BUTTON = A_BUTTON;
	public final static int SPIT_BUTTON = B_BUTTON;
	public final static int ZERO_ENCODER_BUTTON = BACK_BUTTON;
	public final static int LIFT_BOTTOM_HEIGHT_POV = POV_DOWN;
	public final static int LIFT_EXCHANGE_HEIGHT_POV = POV_LEFT;
	public final static int LIFT_TIER2_HEIGHT_POV = POV_RIGHT;
	public final static int LIFT_SWITCH_HEIGHT_POV = POV_UP;
	//public final static int LIFT_TOP_HEIGHT_BUTTON = LB_BUTTON;
	public final static int REACH_BUTTON =  Y_BUTTON;
	public final static int PULL_UP_BUTTON =  X_BUTTON;
	public final static int FLIP_AXIS = RT_AXIS;
	public final static int DIP_AXIS = LT_AXIS;
	public final static int LOCK_BUTTON = RB_BUTTON;
	public final static int UNLOCK_BUTTON = LB_BUTTON;

	
	// Drive train motor channels
	public final static int DRIVE_FRONT_LEFT_CHANNEL = 3;
	public final static int DRIVE_BACK_LEFT_CHANNEL = 4;
	public final static int DRIVE_FRONT_RIGHT_CHANNEL = 2;
	public final static int DRIVE_BACK_RIGHT_CHANNEL = 1;

	// Mouth motor channels
	public final static int MOUTH_RIGHT_CHANNEL = 6;
	public final static int MOUTH_LEFT_CHANNEL = 5;
	public final static int MOUTH_LIMITER_CHANNEL = 0;

	// Lift motor channel
	public final static int LIFT_MOTOR_CHANNEL = 7;
	public final static int LIFT_LIMIT_TOP_CHANNEL = 1;
	public final static int LIFT_LIMIT_BOTTOM_CHANNEL = 2;
	
	//PCM ports
	public final static int COMPRESSOR_PORT = 0;
	public final static int FLIP_PORT = 4;
	public final static int DIP_PORT = 3;
	public final static int LOCK_PORT = 2;
	public final static int UNLOCK_PORT = 5;
	
	
	
	//NavX angles
	public final static double LEFT_WALL_ANGLE = -90.0; //270.0?
	public final static double RIGHT_WALL_ANGLE = 90.0;
	public final static double BACK_WALL_ANGLE = 179.9;
	public final static double FRONT_WALL_ANGLE = 0.0;
	
	
	
	
}

