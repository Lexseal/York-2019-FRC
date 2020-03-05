package frc.robot.commands;
import frc.robot.subsystems.*;

public class AutoTest extends AutoMode {
	//int position = 0;
	//Drive drive;
	//Elevator lifter;
	//Intake intake;
	//double freq;
	//DriverStation station;
	//RecordingReader[] reader;
	//double startTime;
	
	/*private double getCurTime() {
		return System.currentTimeMillis()/1000.0;
	}
	
	private double getRunTime() {
		return getCurTime()-startTime;
	}*/
	
	public AutoTest(Drive _drive, Elevator _lifter, Intake _intake, double _freq) {
		super(_drive, _lifter, _intake, _freq);
		//reader.add(new RecordingReader("timeOptimal"));
		/*reader[0] = new RecordingReader("R_RSwitchToHub");
		reader[1] = new RecordingReader("R_RScaleToHub");
		reader[2] = new RecordingReader("R_PickUpCube");
		reader[3] = new RecordingReader("R_ReturnToHub");
		reader[4] = new RecordingReader("R_HubToRScale");
		reader[5] = new RecordingReader("R_HubToLScale");
		reader[6] = new RecordingReader("R_HubToLSwitch");
		reader[7] = new RecordingReader("R_LSwitch");
		reader[8] = new RecordingReader("R_LScale");
		reader[9] = new RecordingReader("");*/
	}

	// Called just before this Command runs the first time
	public void initialize(int[] plateAssignment) {
		if (!drive.isAlive()) {
			drive.start();
		}
		drive.zeroSensor();
	}

	// Called repeatedly when this Command is scheduled to run
	/*public void execute() {
		System.out.println(reader.length);
		for (int i = 0; i < reader.length; i++) {
			startTime = getRunTime();
			double runTime = getRunTime();
			reader[i].resetCutoff();
			while (runTime <= recordTime && station.isAutonomous() && station.isEnabled()) {
				try {
					Thread.sleep((long)(1000/freq));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}

				runTime = getRunTime();
				double[] vector = reader[i].getVector(runTime);
				//System.out.println("");
				//vector = {runtime, theta, omega1, omega2, s, v, x, y, lift, lIntake, rIntake}
				double angle = vector[1];
				double omega = vector[2];
				double position = vector[3];
				double speed = vector[4];
				double x = vector[5];
				double y = vector[6];
				drive.follow(angle, omega, position, speed, x, y);

				double liftPos = vector[7];
				lifter.updatePosition(liftPos);

				double[] intakeSpeed = {vector[8], vector[9]}; 
				intake.updateSpeed(intakeSpeed);
			}
			if (!station.isAutonomous() || !station.isEnabled()) {
				break;
			}
		}
	}*/

	// Make this return true when this Command no longer needs to run execute()
//	public boolean isFinished() {
//		drive.followFinished();
//		return true;
//	}

	// Called once after isFinished returns true

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}
