package frc.robot.subsystems;

import static frc.robot.Macro.*;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends Thread {
	int port;
	int freq;
	DigitalInput secondStageLanded, thirdStageLanded;
	VictorSPX motor1, motor2;
	double desSpeed, curPos, desPos;
	double desAng;
	double lastTime;
	double I;
	double testKP = 0, testKI = 0, testKD = 0;
	boolean recenterComplete = true;
	DriverStation station;
	Encoder encoder;
	GeneralPID heightController, levelController;
	boolean actionComplete = false;
	ADIS16448_IMU acc;
	double curAng = 0;
	boolean levelingMode = false;
	//FeedForwardController speedController;
	
	public Elevator(int _m1, int _m2, int _encoderPortA, int _encoderPortB, int _switchPin, int _freq, ADIS16448_IMU _imu) {
		freq = _freq;

		motor1 = new VictorSPX(_m1);
		motor2 = new VictorSPX(_m2);
		motor1.setNeutralMode(NeutralMode.Brake);
		motor2.setNeutralMode(NeutralMode.Brake);
		motor1.configOpenloopRamp(0.2, 10);
		motor1.configClosedloopRamp(0.2, 10);
		motor2.configOpenloopRamp(0.2, 10);
		motor2.configClosedloopRamp(0.2, 10);

		motor1.setInverted(true);
		motor2.setInverted(true);

		encoder = new Encoder(_encoderPortA, _encoderPortB, true, Encoder.EncodingType.k4X);
		encoder.setDistancePerPulse(0.0000998876);

		thirdStageLanded = new DigitalInput(_switchPin);
		
		station = DriverStation.getInstance();

		heightController = new GeneralPID(10, 100, 1.5);
		levelController = new GeneralPID(22, 100, 5);
		//speedController = new FeedForwardController(0, 0);
		
		zeroSensor();

		acc = _imu;
	}
	
	public double getDesPos() {
		return desPos;
	}

	public double getCurPos() {
		return encoder.getDistance();
	}

	public double getCurTick() {
		return encoder.get();
	}

	public double getCurSpeed() {
		return encoder.getRate();
	}

	public void setDesPos(double pos) {
		desPos = pos;
	}

	public double[] getCurAng() {
		double accX = acc.getAccelX();
		double accY = acc.getAccelY();
		double accZ = acc.getAccelZ();

		double ang1 = Math.atan2(accX, -accZ)/Math.PI*180;
		double ang2 = Math.atan2(accY, -accZ)/Math.PI*180;

		double[] angs = {ang1, ang2};
		return angs;
	}

	public boolean isFinished() {
        return actionComplete;
    }
	
	public void updateSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		motor2.set(ControlMode.PercentOutput, speed);
	}
	
	public void liftRecenter() {
		recenterComplete = false;
	}
	
	public void updatePosition(double position) {
		curPos = getCurPos();
		desPos = position;
		if (desPos < 0) {
			desPos = 0;
		} else if (desPos > liftMaxHeight) {
			desPos = liftMaxHeight;
		}
	}
	
	public boolean liftIsReady() {
		if (Math.abs(desPos-curPos) < 3) {
			return true;
		}
		return false;
	}
	
	public boolean protectionMode() {
		if (getCurPos() > protectedLiftHeight) {
			return true;
		}
		return false;
	}
	
	public void setConstants(double _kP, double _kI, double _kD) {
		heightController.setPIDConstants(_kP, _kI, _kD);
	}
	
	public void updateDisplacement(double displacement) {
		if (displacement >= 0) {
			desPos += displacement*4;
		} else {
			desPos += displacement*8;
		}
		
		if (desPos < 0) {
			desPos = 0;
		} else if (desPos > liftMaxHeight) {
			desPos = liftMaxHeight;
		}
	}
	
	public boolean zeroSensor() {
		encoder.reset();
		encoder.setDistancePerPulse(0.0000998876);
		SmartDashboard.putBoolean("DB/LED3", true);
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		return true;
	}

	public void setLevel(boolean _state) {
		levelingMode = _state;
	}

	public void setDesAng(double _desAng) {
		desAng = _desAng;
	}
	
	public void run() {
		lastTime = System.currentTimeMillis()/1000;
		curAng = acc.getAngleY();
		desAng = curAng;
		
		while (true) {
			try {
				Thread.sleep(1000/freq);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			curPos = getCurPos();
			
			double error = desPos-curPos;
			double speed = getCurSpeed();
			double currentTime = System.currentTimeMillis()/1000;
			double deltaTime = currentTime - lastTime; //Get deltaTime
			lastTime = currentTime;

			curAng = acc.getAngleY();
			//curAng = curAng*0.8+getCurAng()[0]*0.2;
			SmartDashboard.putString(SDLMotor, ""+curAng);

			//SmartDashboard.putString(SDcurPos, ""+curPos);
			//SmartDashboard.putString(SDdesPos, ""+desPos);
			
			double output = heightController.getOutput(error, speed);
			//SmartDashboard.putString(SDLMotor, ""+getCurPos());

			if (error > 0 && Math.abs(error) < 8) {
				if (output > 30) {
					output = 30;
				}
			}

			if (levelingMode) {
				error = desAng-curAng;
				if (Math.abs(error) < 3) {
					error = 0;
				}
				double dError = acc.getRateY();
				output = levelController.getOutput(error, dError);
				if (output > 0) {
					output /= 5;
				}
			}

			if (!thirdStageLanded.get()) {
				zeroSensor();
				if (output < 0) {
					output = 0;
				}
				SmartDashboard.putString(SDRMotor, "landing!");
			} else {
				SmartDashboard.putString(SDRMotor, "liftOff!");
			}

			updateSpeed(output/100);
			//SmartDashboard.putString("DB/String 4", ""+output);
			//SmartDashboard.putString(SDLMotor, ""+error);
			//SmartDashboard.putString(SDRMotor, ""+output);

		/*if (recenterComplete) {
				if (switchPressed()) {
					if (output < 0) {
						updateSpeed(0);
					} else {
						updateSpeed(output);
					}
					zeroSensor();
				} else {
					SmartDashboard.putBoolean("DB/LED3", false);
					updateSpeed(output);
				}
			} else {
				if (switchPressed()) {
					recenterComplete = true;
					zeroSensor();
					desPos = curPos;
					updateSpeed(0);
				} else {
					updateSpeed(-0.2);
				}
			}
						if (!station.isEnabled()) {
								desPos = curPos;
							}*/
			//System.out.println(desPos+"   "+curPos+"   "+motor.getSelectedSensorPosition(0) + "   " +output);

			if (Math.abs(error) < 2) {
				actionComplete = true;
			} else {
				actionComplete = false;
			}
		}
	}
}
