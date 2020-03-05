package frc.robot;

/**
 * The file that stores all the static final variables
 */
public class Macro {
	public static final boolean betaFeature = false;
	public static final boolean driveTestMode = false;
	public static final boolean liftTestMode = false;
	public static final boolean wristTestMode = false;
	
	/*
	 * xbox controller mapping
	 */
	public static final int LEFT_X = 0;
	public static final int THROTTLE = 1;
	public static final int LEFT_UP = 2;
	public static final int RIGHT_UP = 3;
	public static final int TURN = 4;
	public static final int RIGHT_Y = 5;
	public static final int MAX = 16;
	
	public static final int A = 1;
	public static final int B = 2;
	public static final int X = 3;
	public static final int Y = 4;
	public static final int LB = 5;
	public static final int RB = 6;
	public static final int BACK = 7;
	public static final int START = 8;
	public static final int leftAxisButton = 9;
	public static final int rightAxisButton = 10;
	
	public static final int INTAKE_POS_BUTTON = 1;
	public static final int SWITCH_POS_BUTTON = 2;
	public static final int SCALE_POS_BUTTON = 3;
	public static final int LIFT_RECENTER = 4;
	public static final int INTAKE_BUTTON = 0;
	public static final int SPIT_OUT_BUTTON = 0;
	
	/*
	 * encoder channel
	 */
	public static final int encoderLeft = 0;
	public static final int encoderRight = 2;
	
	/*
	 * PID constants (I forgot which are which... should have kept
	 * better organization :( )
	 */
	public static final double maxPID = 100;
	public static final double constkP = 1.2; //2.4;
	public static final double constkI = 0.1; //2;
	public static final double constkD = 0; //0.24; *0.7
	public static final double ICapMultiplier = 10;
	public static final double turnGain1 = 0.2;
	public static final double turnkP1 =  0.2;
	public static final double driveGain1 = 40;
	public static final double drivekP1 =  30;
	public static final double turnGain2 = 0.225;
	public static final double turnkP2 =  0.75;
	public static final double driveGain2 = 30;
	public static final double drivekP2 =  18;
	
	/* 
	 * Robot max characteristics
	 */
	public static final double maxOmega = 270; // maximum turning rate (degrees/sec)
	
	public static final double maxSpeed = 3; // m/s
	public static final double ticksPerRev = 4096; // encoder tick per rev
	public static final double wheelCircumfrence = 0.3192; // meters
	
	public static final double liftHeightPerRev = 12.543; // cm/rev
	public static final double liftkP = 0.08;
	public static final double liftkI = 1;
	public static final double liftkD = 0.00008;
	public static final double liftMaxSpeed = 2;
	public static final double protectedLiftHeight = 140;
	public static final double liftHome = 0;
	public static final double liftSwitchHeight = 105;
	public static final double liftMaxHeight = 233.5;
	public static final double secondStageLanding = 134;
	public static final double liftHeight0 = 0;
	public static final double liftHeight1 = 0;
	public static final double liftHeight2 = 0;
	public static final double liftHeight3 = 0;
	
	/*
	 * autonomous parameters
	 */
	public static final String switchFirst = "switchFirst";
	public static final String scaleFirst = "scaleFirst";
	public static final String leftStart = "L";
	public static final String middleStart = "M";
	public static final String rightStart = "R";

	public static final double recordTime = 15;

	/*
	 * Misc debugging output
	 */
	public static final String SDcurAng = "DB/String 0";
	public static final String SDdesAng = "DB/String 1";
	public static final String SDcurPos = "DB/String 2";
	public static final String SDdesPos = "DB/String 3";
	public static final String SDturnGain = "DB/String 4";
	public static final String SDturnkP = "DB/String 5";
	public static final String SDdriveGain = "DB/String 6";
	public static final String SDdrivekP = "DB/String 7";
	public static final String SDkP = "DB/String 5";
	public static final String SDkI = "DB/String 6";
	public static final String SDkD = "DB/String 7";
	public static final String SDLMotor = "DB/String 8";
	public static final String SDRMotor = "DB/String 9";
}
