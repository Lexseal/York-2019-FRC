/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static frc.robot.Macro.*;

import com.analog.adis16448.frc.ADIS16448_IMU;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    //PWM pwm = new PWM(0);
    AutoMode[] modes = new AutoMode[5];
	AutoMode autoMode;

	SendableChooser<String> priorityChooser = new SendableChooser<String>();
	SendableChooser<String> positionChooser = new SendableChooser<String>();

	int axisList[] = { LEFT_X, THROTTLE, LEFT_UP, RIGHT_UP, TURN, RIGHT_Y }; //the list of axis you are interested to track
	int buttonList[] = { A, B, X, Y, LB, RB, BACK, START, leftAxisButton, rightAxisButton }; //the list of buttons you are interested to track
	Controller driveStick = new Controller(0, axisList, buttonList, 1, 18, 200, 2); //driving xbox controller with port 0, 1% deadband and 18% cutoff running at 200Hz. The return value is x^1.6
	//Controller controlStick = new Controller(1, axisList, buttonList, 1, 20, 200, 1.6); //secondary xbox controller with port 0, 1% deadband and 20% cutoff running at 200Hz. The return value is x^1.6

	ADIS16448_IMU imu = new ADIS16448_IMU();

	int leftMotors[] = { 3, 5 };
	int rightMotors[] = { 2, 4 }; //motor CAN IDs
	Drive drive = new Drive(leftMotors, rightMotors, 200, imu); //200Hz

	Compressor compressor = new Compressor();
	DoubleSolenoid walkerExtension = new DoubleSolenoid(2, 3);
	DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);
	DoubleSolenoid deployHatch = new DoubleSolenoid(4, 5);

	Elevator lift = new Elevator(8, 11, 2, 3, 0, 200, imu); //lift with motor, limit switch 9 at 200Hz

	int[] wristMotors = { 6, 9 }; //8 is lift 7 is blank
	Wrist wrist = new Wrist(4, 5, 1, wristMotors, 100);

	Intake intake = new Intake(10, 200);

	//Record recorder;
	//Thread recordingThread = new Thread();

	boolean lastLB = false;
	boolean lastRB = false;

	boolean lastAxisLButton = false;
	boolean lastAxisRButton = false;
	boolean downShift = false;

	boolean lastA = false;
	boolean lastB = false;
	boolean lastY = false;
	boolean lastX = false;

	int curLiftStage = -1;
	int curWristStage = 0;
	boolean runCompressor = true;

	boolean walkerExtended = false;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    priorityChooser.setDefaultOption("Switch", switchFirst);
	priorityChooser.addOption("Scale", scaleFirst);
		
	positionChooser.setDefaultOption("Left Start", leftStart);
	positionChooser.addOption("Middle Start", middleStart);
	positionChooser.addOption("Right Start", rightStart);

	SmartDashboard.putData("Priority Chooser", priorityChooser);
	SmartDashboard.putData("Position Chooser", positionChooser);

	compressor.setClosedLoopControl(true);
	compressor.start();
	
	/*modes[0] = new AutoSwitchFromLeft(drive, lift, intake, 100);
	modes[1] = new AutoSwitchFromMiddle(drive, lift, intake, 100);
	modes[2] = new AutoSwitchFromRight(drive, lift, intake, 100);
	modes[3] = new AutoScaleFromLeft(drive, lift, intake, 100);
	modes[4] = new AutoScaleFromRight(drive, lift, intake, 100); //initialize all 5 auto modes here*/

	intake.start();
	lift.start();
	wrist.start();
	//stream.start(); //start intake, lift, and streaming service
  }

  @Override
	public void disabledInit() {
		/*if (!modes[0].isFresh()) {
			modes[0] = new AutoSwitchFromLeft(drive, lift, intake, 100);
		}
		if (!modes[1].isFresh()) {
			modes[1] = new AutoSwitchFromMiddle(drive, lift, intake, 100);
		}
		if (!modes[2].isFresh()) {
			modes[2] = new AutoSwitchFromRight(drive, lift, intake, 100);
		}
		if (!modes[3].isFresh()) {
			modes[3] = new AutoScaleFromLeft(drive, lift, intake, 100);
		}
		if (!modes[4].isFresh()) {
			modes[4] = new AutoScaleFromRight(drive, lift, intake, 100);
		}*/
	}

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
	teleopInit();
	//AutoMode auto = new AutoMode(drive, lift, intake, 200);
	//auto.initialize();
	//auto.execute();
    /*String priority = priorityChooser.getSelected();
	String position = positionChooser.getSelected();
	
	System.out.println(position);
	if (priority == switchFirst) {
		if (position == leftStart) {
			autoMode = modes[0];
			System.out.println("mode0");
		} else if (position == middleStart) {
			autoMode = modes[1];
			System.out.println("mode1");
		} else if (position == rightStart) {
			autoMode = modes[2];
			System.out.println("mode2");
		}
	} else if (priority == scaleFirst) {
		if (position == leftStart) {
			autoMode = modes[3];
			System.out.println("mode3");
		} else if (position == rightStart) {
			autoMode = modes[4];
			System.out.println("mode4");
		}
	} //get the desired auto mode

	DriverStation station = DriverStation.getInstance();
	String platePlacement = station.getGameSpecificMessage();
	//platePlacement = SmartDashboard.getString(SDkD, "");

	int[] platePos = { 0, 0 };
	if (platePlacement.length() > 0) {
		for (int i = 0; i < 2; i++) {
			if (platePlacement.charAt(i) == 'L') {
				platePos[i] = -1;
			} else if (platePlacement.charAt(i) == 'R') {
				platePos[i] = 1;
			}
		}
	} //get the position of plates. {1, -1} means the switch on the right but the scale is on the left
	
	System.out.println(platePos[0] +" "+ platePos[1]);
	if (autoMode != null && autoMode.isFresh()) {
		autoMode.initialize(platePos);
		autoMode.execute();
		autoMode.isFinished();
	}*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
	teleopPeriodic();
  }

  @Override
	public void teleopInit() {
		if (autoMode != null) {
			autoMode.isFinished();
		}
		
		if (!driveStick.isAlive()) {
			driveStick.start();
		}
		//if (!controlStick.isAlive()) {
		//	controlStick.start();
		//}
		if (!drive.isAlive()) {
			drive.start();
		}
		drive.zeroSensor();
	}

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
	/**
	 * Drive
	 */
	double turnGain = turnGain1;
	double turnkP = turnkP1;
	double driveGain = driveGain1;
	double drivekP = drivekP1;

	boolean curAxisLButton = driveStick.getButton(leftAxisButton);
	boolean curAxisRButton = driveStick.getButton(rightAxisButton);
	if (curAxisLButton != lastAxisLButton && curAxisLButton == true) {
		gearShift.set(DoubleSolenoid.Value.kForward);
		Timer.delay(0.01);
	} else if (curAxisRButton != lastAxisRButton && curAxisRButton == true) {
		gearShift.set(DoubleSolenoid.Value.kReverse);
		Timer.delay(0.01);
		/*turnGain = turnGain2;
		turnkP = turnkP2;
		driveGain = driveGain2;
		drivekP = drivekP2;*/
	} else {
		gearShift.set(DoubleSolenoid.Value.kOff);
	}
	lastAxisLButton = curAxisLButton;
	lastAxisRButton = curAxisRButton;

	if (driveTestMode) { // Set constants in test mode
		turnGain = Double.parseDouble(SmartDashboard.getString(SDturnGain, ""));
		turnkP = Double.parseDouble(SmartDashboard.getString(SDturnkP, ""));
		driveGain = Double.parseDouble(SmartDashboard.getString(SDdriveGain, ""));
		drivekP = Double.parseDouble(SmartDashboard.getString(SDdrivekP, ""));
	}

	drive.setConstants(turnGain, turnkP, driveGain, drivekP);
	drive.updateVelocity(-driveStick.getAxis(THROTTLE), driveStick.getAxis(TURN));

	/**
	 * Elevator
	 */
	if (liftTestMode) {
		double kP = Double.parseDouble(SmartDashboard.getString(SDkP, ""));
		double kI = Double.parseDouble(SmartDashboard.getString(SDkI, ""));
		double kD = Double.parseDouble(SmartDashboard.getString(SDkD, ""));
		lift.setConstants(kP, kI, kD);
	}
	
	boolean curLB = driveStick.getButton(LB);
	boolean curRB = driveStick.getButton(RB);
	if (curLB != lastLB && curLB == true) {
		lift.setConstants(10, 100, 1.5);
		wrist.updateConstants(3, 100, 1.45);
		lift.setLevel(false);

		curLiftStage -= 1;

		if (curLiftStage < -1) {
			curLiftStage = -1;
		}

		curWristStage = 0;
	} else if (curRB != lastRB && curRB == true) {
		lift.setConstants(10, 100, 1.5);
		wrist.updateConstants(3, 100, 1.45);
		lift.setLevel(false);
		curLiftStage += 1;
		curWristStage = 1;

		if (curLiftStage > 3) {
			curLiftStage = 3;
		}
		
		if (curLiftStage == 0) {
			curWristStage = 0;
		} else if (curLiftStage == 2) {
			curWristStage = 3;
		} else if (curLiftStage == 3) {
			curWristStage = 2;
		}
	}
	
	lastLB = curLB;
	lastRB = curRB;

	double curDesPos = -1;
	if (curLiftStage == -1) {
		curDesPos = -1;
	} else if (curLiftStage == 0) {
		curDesPos = 4; //5.5
	}else if (curLiftStage == 1) {
		curDesPos = 10;
	} else if (curLiftStage == 2) {
		curDesPos = 30;
	} else if (curLiftStage == 3) {
		curDesPos = 35;
	}
	if (driveStick.getButton(X)) {
		curDesPos -= 4.5;
		deployHatch.set(DoubleSolenoid.Value.kReverse);
	}

	lift.setDesPos(curDesPos);

	if (curLiftStage == 0 && driveStick.getPOV() == 315) {
		//lift.home();
	}

	//lift.updateSpeed(driveStick.getAxis(RIGHT_UP)-driveStick.getAxis(LEFT_UP));
	if ((driveStick.getAxis(LEFT_UP)-driveStick.getAxis(RIGHT_UP)) == 0) {
		intake.updateSpeed(0.09);
	} else {
		intake.updateSpeed(driveStick.getAxis(LEFT_UP)-driveStick.getAxis(RIGHT_UP));
	}

	int pov = driveStick.getPOV();
	if (pov == 135 && driveStick.getButton(BACK)) {
		if (walkerExtended == false) {
			walkerExtended = true;
			imu.reset();
		}
		walkerExtension.set(DoubleSolenoid.Value.kReverse);
		wrist.updateConstants(20, 100, 0.08);
		Timer.delay(0.01);
		lift.setDesAng(0);
		lift.setLevel(true);
		curLiftStage = 0;
		drive.activateLittleWheels();
		//elevator goes down liftStage -1 with power
		//elevator wrist hold wristStage 3 with power
		//walker
	} else if (pov == 225) {
		lift.setLevel(false);
		walkerExtended = false;
		drive.deActivateLittleWheels();
		walkerExtension.set(DoubleSolenoid.Value.kForward);
		Timer.delay(0.01);
	} else {
		walkerExtension.set(DoubleSolenoid.Value.kOff);
	}
	//SmartDashboard.putString(SDLMotor, ""+pov);

	if (!driveStick.getButton(X)) {
		boolean curY = driveStick.getButton(Y);
		if (curY) {
			deployHatch.set(DoubleSolenoid.Value.kReverse);
			Timer.delay(0.01);
		} else if (curY != lastY && curY == false) {
			deployHatch.set(DoubleSolenoid.Value.kForward);
			Timer.delay(0.01);
		} else {
			deployHatch.set(DoubleSolenoid.Value.kOff);
		}
		lastY = curY;
	}

	boolean start = driveStick.getButton(START);
	boolean stop = driveStick.getButton(BACK);
	if (stop) {
		compressor.stop();
	} else if (start) {
		compressor.start();
		//System.out.print("start");
	}

	if (wristTestMode) {
		double kP = Double.parseDouble(SmartDashboard.getString(SDkP, ""));
		double kI = Double.parseDouble(SmartDashboard.getString(SDkI, ""));
		double kD = Double.parseDouble(SmartDashboard.getString(SDkD, ""));
		wrist.updateConstants(kP, kI, kD);
	}
	boolean curA = driveStick.getButton(A);
	boolean curB = driveStick.getButton(B);
	if (curA != lastA && curA == true) {
		curWristStage += 1;
		lift.setLevel(false);
		wrist.updateConstants(3, 100, 1.45);
	} else if (curB != lastB && curB == true) {
		curWristStage -= 1;
		lift.setLevel(false);
		wrist.updateConstants(3, 100, 1.45);
	}
	if (curWristStage < 0) {
		curWristStage = 0;
	} else if (curWristStage > 4) {
		curWristStage = 4;
	}
	lastA = curA;
	lastB = curB;

	if (driveStick.getButton(X)) {
		wrist.setDesAng(90);
	}
	if (curWristStage == 0) {
		wrist.setDesAng(-10);
	} else if (curWristStage == 1) {
		wrist.setDesAng(80);
	} else if (curWristStage == 2) {
		wrist.setDesAng(135);
	} else if (curWristStage == 3) {
		wrist.setDesAng(195);
	} else if (curWristStage == 4) {
		wrist.setDesAng(220);
	}
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
