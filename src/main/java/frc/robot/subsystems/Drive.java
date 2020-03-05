package frc.robot.subsystems;

import static frc.robot.Macro.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Drive extends Thread {
	int[] leftMotorNum;
	int[] rightMotorNum; //Motor CAN ports.
	TalonSRX[] motor = new TalonSRX[MAX];
	VictorSPX[] littleMotor = new VictorSPX[2];
	//PWMVictorSPX[] motor = new PWMVictorSPX[4];
	/*In a 4-motor config, 0 and 1 are left motors while 2 and 3 are right motors.
	  In a 6-motor config, 0, 1, and 2 are left while 3, 4, and 5 are right.*/
	ADIS16448_IMU imu;
	DriverStation station = DriverStation.getInstance();
	FeedForwardController turnController;
	FeedForwardController speedController;
	
	double curAng, desAng, curOmega, desOmega, lastOmega, curAlpha;
	double desPos, curSpeed, desSpeed, curPos;
	double[] desDisp = new double[2], desVelocity = new double[2];
	double[] curDisp = new double[2], curVelocity = new double[2];
	double throttle = 0, lastThrottle = 0;
	double desX, x, desY, y; //robot position
	double lastTime, deltaTime;
	double freq;
	double I = 0; //integral term
	double lastTurn = 0; //register the turn command at last cycle to determine if breaking is needed this cycle
	double liftHeight = 0;
	double liftRestrictionMultiplier = 0;
	double restrictionMultiplier = 1;
	boolean isFollowMode = false;
	boolean isRecording = false;
	boolean littleWheelsActive = false;
	
	public Drive(int _left[], int _right[], double _freq, ADIS16448_IMU _imu) {
		leftMotorNum = _left;
		rightMotorNum = _right;
		for (int i = 0; i < _left.length; i++) {
			motor[i] = new TalonSRX(_left[i]); //Init left motors using CAN.
			motor[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
			motor[i].setInverted(false);
			motor[i].setSensorPhase(true);
		}
		for (int i = 0; i < _right.length; i++) {
			motor[i+_left.length] = new TalonSRX(_right[i]); //Init right motors.
			motor[i+_left.length].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
			motor[i+_left.length].setInverted(true);
			motor[i+_left.length].setSensorPhase(true);
		}
		
		for (int i = 0; i < _left.length+_right.length; i++) {
			motor[i].setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10); // modify the encoder refresh rate
			
			motor[i].configNominalOutputForward(0, 10);
			motor[i].configNominalOutputReverse(0, 10);
			motor[i].configPeakOutputForward(1, 10);
			motor[i].configPeakOutputReverse(-1, 10);
			
			motor[i].config_kF(0, 0, 10);
			motor[i].config_kP(0, 0.24, 10);
			motor[i].config_kI(0, 0, 10); 
			motor[i].config_kD(0, 0.005, 10);
			
			motor[i].configOpenloopRamp(0.1, 10);
			motor[i].configClosedloopRamp(0.1, 10);
		}
		
		imu = _imu;
		imu.calibrate(); //Init gyro. 

		turnController = new FeedForwardController(0.15, 0.5);
		speedController = new FeedForwardController(driveGain1, drivekP1);
		
		freq = _freq;

		littleMotor[0] = new VictorSPX(7);
		littleMotor[0].setInverted(true);
		littleMotor[1] = new VictorSPX(12);
		littleMotor[1].setInverted(false);
	}
	
	public double[] getcurVelocity() {
		double[] encoderReadOut = {(double)motor[0].getSelectedSensorVelocity(0)*10.0/ticksPerRev*wheelCircumfrence, (double)motor[2].getSelectedSensorVelocity(0)*10.0/ticksPerRev*wheelCircumfrence};
		return encoderReadOut;
	}
	
	private double[] getcurDisp() {
		double[] encoderReadOut = {(double)motor[0].getSelectedSensorPosition(0)/ticksPerRev*wheelCircumfrence, (double)motor[2].getSelectedSensorPosition(0)/ticksPerRev*wheelCircumfrence};
		return encoderReadOut;
	}
	
	private void updateCoordinate() {
		double dc = curSpeed*deltaTime;
		x += Math.sin(curAng*3.1415926/180)*dc;
		y += Math.cos(curAng*3.1415926/180)*dc;
	}

	public void activateLittleWheels() {
		littleWheelsActive = true;
	}

	public void deActivateLittleWheels() {
		littleWheelsActive = false;
	}
	
	private void updateMotor(double leftOutput, double rightOutput, ControlMode mode) {
		//SmartDashboard.putString(SDLMotor, ""+leftOutput);
		//SmartDashboard.putString(SDRMotor, ""+rightOutput);
		/*motor[0].set(mode, leftOutput*maxRevPer100ms*wheelMultiplier);
		motor[1].set(ControlMode.Follower, motor[0].getDeviceID());
		motor[2].set(mode, rightOutput*maxRevPer100ms*wheelMultiplier);
		motor[3].set(ControlMode.Follower, motor[2].getDeviceID());*/
		motor[0].set(mode, leftOutput);
		motor[1].set(ControlMode.Follower, motor[0].getDeviceID());
		motor[2].set(mode, rightOutput);
		motor[3].set(ControlMode.Follower, motor[2].getDeviceID());

		if (littleWheelsActive) {
			littleMotor[0].set(mode, leftOutput*1);
			littleMotor[1].set(mode, rightOutput*1);
		}
		/*motor[0].set(leftOutput);
		motor[1].set(leftOutput);
		motor[2].set(-rightOutput);
		motor[3].set(-rightOutput);*/
	}
	
	public void updateVelocity(double _speed, double _turn) {
		desSpeed = _speed*maxSpeed;
		desOmega = _turn*maxOmega;
		lastTurn = _turn;
		followFinished();
	}
	
	public void updateDisplacement(double _angle, double _displacement) {
		//achieve angle
		desAng = curAng + _angle; 
		System.out.println("ang and disp" + _angle +" "+ _displacement + " desiAng " + desAng);
		double startTime = System.currentTimeMillis()/1000.0;
		while (Math.abs(desAng-imu.getAngle())>1 || (System.currentTimeMillis()/1000.0-startTime)<1000 || (station.isAutonomous() && station.isEnabled())) {
		}
		
		//achieve displacement
		/*curPos = (motor[0].getSelectedSensorPosition(0)+motor[2].getSelectedSensorPosition(0))/2;
		curPos = curPos/wheelMultiplier*wheelCircumfrence;*/
		//desPos = curPos+_displacement;
		startTime = System.currentTimeMillis()/1000.0;
		while (station.isAutonomous() && station.isEnabled()) {
			/*curPos = (motor[0].getSelectedSensorPosition(0)+motor[2].getSelectedSensorPosition(0))/2;
			curPos = curPos/wheelMultiplier*wheelCircumfrence;*/
			//double error = desPos-curPos;
			
			//if (Math.abs(error) < 0.05 /*|| (System.currentTimeMillis()/1000.0-startTime)>_displacement*2000*/) {
			//	updateVelocity(0, 0);
			//	break;
			//} else {
				//double power = Math.abs(0.25-Math.pow((error-_displacement/2), 4)/(0.625*Math.pow(_displacement, 4)));
				//double power = Math.abs(0.6-Math.pow((error-_displacement/2), 2)/(0.71*Math.pow(_displacement, 2)));
			//	double iniP = 0.2, finP = 0.35+Math.abs(_displacement)*0.1;
			//	double dter = (16/Math.pow(_displacement, 2)+2);
			//	double a = (finP-iniP)*dter;
			//	double b = +Math.abs(_displacement)/2;
			//	double c = iniP-a/(2+Math.pow(b, 2));
				//double power = a/(2+Math.pow((Math.abs(error)-b), 2))+c;
				
				//if (error > 0) {
				//	System.out.println(power+" "+curPos+" "+error);
				//	updateVelocity(power, 0);
				//} else {
				//	System.out.println(power+" "+curPos+" "+error);
				//	updateVelocity(-power, 0);
				//}
			//}
		}
	}
	
	public void updatePosition(double _x, double _y) {
		double dx = _x-x;
		double dy = _y-y;
		double theta = -((Math.atan2(dy, dx))*(180/3.1416)-90);
		double correctAng = (curAng)%360;
		double dTheta = theta-correctAng;
		double displacement = Math.hypot(dx, dy);
		System.out.println("dTheta" + dTheta +" "+ displacement);
		updateDisplacement(dTheta, displacement);
	}
	
	public void followFinished() {
		isFollowMode = false;
	}
	
	public void follow(double speedL, double speedR, double leftPos, double rightPos) {
		isFollowMode = true;
		desVelocity[0] = speedL;
		desVelocity[1] = speedR;
		desDisp[0] = leftPos;
		desDisp[1] = rightPos;
		
		//desX = _x;
		//desY = _y;
	}
	
	public void setConstants(double _turnGain, double _turnkP, double _driveGain, double _drivekP) {
		turnController.setConstants(_turnGain, _turnkP);
		speedController.setConstants(_driveGain, _drivekP);
	}
	
	public boolean zeroSensor() {
		imu.reset();
		motor[0].setSelectedSensorPosition(0, 0, 10);
		motor[2].setSelectedSensorPosition(0, 0, 10);
		
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		curAng = imu.getAngle();
		desAng = curAng;
		x = 0;
		y = 0;
		return true;
	}
	
	public void restrictedAcc() {
		restrictionMultiplier = 0.3;
	}
	
	public void normalAcc() {
		restrictionMultiplier = 1;
	}
	
	public void setRecordingStat(boolean value) {
		isRecording = value;
	}
	
	public void run() {
		desAng = curAng;
		
		x = 0;
		y = 0;
		
		lastTime = System.currentTimeMillis()/1000.0; //Init lastTime for integral calculation.
		lastOmega = imu.getRate();
		
		while (true) {
			try {
				Thread.sleep((long)(1000/freq));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			double curTime = System.currentTimeMillis()/1000.0;
			deltaTime = curTime - lastTime; //Get deltaTime
			lastTime = curTime;

			curAng = imu.getAngle();
			curOmega = imu.getRate(); //Get omega
			curAlpha = (curOmega-lastOmega)/deltaTime;
			//SmartDashboard.putString(SDcurAng, ""+curAng);
			
			curDisp = getcurDisp();
			curPos = (curDisp[0]+curDisp[1])/2;
			curVelocity = getcurVelocity();
			curSpeed = (curVelocity[0]+curVelocity[1])/2.0;
			//SmartDashboard.putString(SDLMotor, "l"+curDisp[0]);
			//SmartDashboard.putString(SDRMotor, "r"+curDisp[1]);
			//curSpeed = getCurSpeed();
			//SmartDashboard.putString(SDcurPos, ""+curSpeed);
			
			//updateCoordinate();
			
			if (isFollowMode && station.isAutonomous()) {
				double[] velocityErr = {desVelocity[0]-curVelocity[0], desVelocity[1]-curVelocity[1]};
				double[] dispErr = {desDisp[0]-curDisp[0], desDisp[1]-curDisp[1]};
				
				double[] output = new double[2];
				output[0] = speedController.getOutput(desVelocity[0], velocityErr[0]);
				output[1] = speedController.getOutput(desVelocity[1], velocityErr[1]);
				
				//output[0] += 100*dispErr[0];
				//output[1] += 100*dispErr[1];

				//System.out.println(desDisp[0]+" "+ desDisp[1]);
				updateMotor(output[0]/100, output[1]/100, ControlMode.PercentOutput);
				
				double angleCorrection = 180/3.1415925*Math.atan2(desX-x, desY-y)-curAng;
				double hypoCorrection = Math.hypot(desX-x, desY-y);
				angleCorrection *= Math.pow(hypoCorrection, 1);
				if (angleCorrection > 3){
					angleCorrection = 3;
				} else if (angleCorrection < -3) {
					angleCorrection = -3;
				}
				/*if (getCurSpeed() < 0) {
					angleCorrection *= -1;
				}*/
				//System.out.println(angleCorrection);
				
				double omegaErr = desOmega-curOmega;
				//double outputFromOmega = desOmega*0.0052+omegaErr*0.0016;
				//0.0052 0.0017
				//outputFromOmega = 0;
				
				double angErr = desAng-curAng;
				angErr += angleCorrection;
				//double kP = Double.parseDouble(SmartDashboard.getString(SDkP, ""));
				//double outputFromAng = angErr*0.016;
				//0.0005
				//outputFromAng = 0;
				//System.out.println(omegaErr+" "+angErr);
				
				//double outputL = outputFromSpeed+outputFromPos+outputFromAng+outputFromOmega;
				//double outputR = outputFromSpeed+outputFromPos-outputFromAng-outputFromOmega;
				//double outputL = outputFromSpeed+outputFromPos+updatePID(angErr, curOmega, deltaTime)/100+omegaErr*0.003;
				//double outputR = outputFromSpeed+outputFromPos-updatePID(angErr, curOmega, deltaTime)/100-omegaErr*0.003;
				
				//updateMotor(outputL, outputR, ControlMode.Velocity);
				//System.out.println(outputL+" "+outputR);
			} else {
				double angError = desAng-curAng; //Get angle error
				angError = angError%360;
				if (angError > 180)	 {
					angError -= 360;
				} else if (angError < -180) {
					angError += 360;
				}
				//System.out.println(desOmega);
				double omegaError = desOmega-curOmega;
				double speedError = desSpeed-curSpeed;
				//SmartDashboard.putString(SDAngErr, "desOmega: "+(int)desOmega+" "+(int)omegaError+" "+(int)imu.getRate());
				//SmartDashboard.putString(SDAngErr, ""+angError);
				
				//double output = updatePID(angError, curOmega, deltaTime);
				double outputThrottle = speedController.getOutput(desSpeed, speedError);
				SmartDashboard.putString(SDcurAng, ""+(speedError));//+" "+((int)((outputI)*100))/100.0+ " "+((int)((outputD)*100))/100.0);
				outputThrottle /= 100;
				double outputTurn = turnController.getOutput(desOmega, omegaError);
				SmartDashboard.putString(SDdesAng, ""+((int)((omegaError)*100))/100.0);//+" "+((int)((outputI)*100))/100.0+ " "+((int)((outputD)*100))/100.0);
				outputTurn /= 100;
				
				lastThrottle = throttle;
				
				if (isRecording) {
					updateMotor(0, 0, ControlMode.PercentOutput);
				} else {
					updateMotor(outputThrottle+outputTurn, outputThrottle-outputTurn, ControlMode.PercentOutput);
				}
			}
			lastOmega = curOmega;
		}
	}

	public void setLiftHeight(double curPos2) {
		liftHeight = curPos2;
	}
}
