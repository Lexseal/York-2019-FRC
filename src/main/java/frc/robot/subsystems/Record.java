package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static frc.robot.Macro.*;

public class Record extends Thread{
	double freq;
	double startTime;
	double[] motion = new double[5+3];
	double[] controlServices = new double[3];
	double headingBias, positionBias, xBias, yBias;
	Drive drive;
	Elevator lifter;
	Intake intake;
	String pathToFile;
	FileWriter writer;
	
	public Record(String fileName, Drive _drive, Elevator _lifter, Intake _intake, double _freq) {
		freq = _freq;
		String path = new String("/home/lvuser/recordings/");
		File folder = new File(path);
		if (!folder.exists()) {
			folder.mkdirs();
		}
		pathToFile = new String(path+fileName+".csv");
		drive = _drive;
		lifter = _lifter;
		intake = _intake;
	}
	
	private double getCurTime() {
		return System.currentTimeMillis()/1000.0;
	}
	
	private double getRunTime() {
		return getCurTime()-startTime;
	}
	
	public double[] adjustForBias(double[] motion) {
		motion[0] -= headingBias;
		motion[2] -= positionBias;
		motion[5] -= xBias;
		motion[6] -= yBias;
		/*if (motion[4] == 0) {
			motion[3] = 0;
		}*/
		return motion;
	}
	
	public void run() {
		try {
			writer = new FileWriter(pathToFile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//motion = drive.getAllSensorInfo();
		headingBias = motion[0];
		positionBias = motion[2];
		xBias = motion[5];
		yBias = motion[6];
		
		startTime = getCurTime();
		double runTime = getRunTime();
		double lastAngle = 0;
		double lastTime = 0;
		while (runTime<=recordTime && writer!=null && drive!=null) {
			System.out.println(""+runTime);
			
			try {
				Thread.sleep((long)(1000/freq));
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			runTime = getRunTime();
			//motion = drive.getAllSensorInfo();
			motion = adjustForBias(motion);
			
			double liftPos = lifter.getDesPos();
			//double[] intakeSpeed = intake.getSpeed();
			controlServices[0] = liftPos;
			//controlServices[1] = intakeSpeed[0];
			//controlServices[2] = intakeSpeed[1];
			
			try {
				writer.append(runTime+", "); //runtime
				writer.append(motion[0]+", "); //theta
				writer.append(motion[1]+", "); //gyro omega
				double deltaTime = runTime-lastTime;
				lastTime = runTime;
				writer.append((motion[0]-lastAngle)/deltaTime+", "); //calculated omega
				lastAngle = motion[0];
				writer.append(motion[2]+", "); //s
				writer.append(motion[3]+", "); //v
				writer.append(motion[5]+", "); //x
				writer.append(motion[6]+", "); //y
				
				writer.append(controlServices[0]+", "); //lift height in cm
				writer.append(controlServices[1]+", "); //left intake speed
				writer.append(controlServices[2]+"\n"); //right intake speed
				
				//writer = {runtime, theta, omega1, omega2, s, v, x, y, lift, lIntake, rIntake}
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		if (writer != null) {
			try {
				writer.flush();
				writer.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
