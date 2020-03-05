package frc.robot.commands;

import static frc.robot.Macro.recordTime;

import java.util.ArrayList;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RecordingReader;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoMode {
	protected Drive drive;
	protected Elevator lift;
	protected Intake intake;
	protected double freq;
	protected DriverStation station;
	protected ArrayList<RecordingReader> reader;
	protected RecordingReader leftWheel, rightWheel;
	protected double startTime;
	protected boolean fresh = true;

	public boolean isFresh() {
		return fresh;
	}
	
	protected double getCurTime() {
		return System.currentTimeMillis()/1000.0;
	}

	protected double getRunTime() {
		return getCurTime()-startTime;
	}

	public AutoMode(Drive _drive, Elevator _lift, Intake _intake, double _freq) {
		freq = _freq;
		drive = _drive;
		lift = _lift;
		intake = _intake;
		station = DriverStation.getInstance();
		reader = new ArrayList<RecordingReader>(0);
		leftWheel = new RecordingReader("left");
		rightWheel = new RecordingReader("right");
	}

	// Called  before running execute
	public void initialize() {
		System.out.println("initialized");
		if (!drive.isAlive()) {
			drive.start();
		}
		drive.zeroSensor();
	}

	// Called once
	public void execute() {
		System.out.println(reader.size());
		startTime = getRunTime();
		double runTime = getRunTime();
		while (runTime < leftWheel.getTotalTime() && station.isAutonomous() && station.isEnabled()) {
			try {
				Thread.sleep((long)(1000/freq));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			runTime = getRunTime();
			double[] left = leftWheel.getVector(runTime);
			double[] right = rightWheel.getVector(runTime);
			//System.out.println("");
			//vector = {dt, x, y, s, v, a, j, omega}
			double leftPos = left[3];
			double rightPos = right[3];
			double leftSpeed = left[4];
			double rightSpeed = right[4];
			System.out.println(runTime+" "+leftSpeed+" "+ rightSpeed);
			drive.follow(leftSpeed, rightSpeed, leftPos, rightPos);
			//intake.updateSpeed(intakeSpeed);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		drive.followFinished();
		fresh = false;
		return true;
	}

	// Called once after isFinished returns true
}