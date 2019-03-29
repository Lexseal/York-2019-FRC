package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber {
	int port;
	TalonSRX motor;
	
	public Climber(int _port) {
		port = _port;
		
		motor = new TalonSRX(port);
		
		motor.setInverted(false);
		motor.setSensorPhase(true);
		
		//motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		//motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);
		
		motor.configNominalOutputForward(0, 10);
		motor.configNominalOutputReverse(0, 10);
		motor.configPeakOutputForward(1, 10);
		motor.configPeakOutputReverse(-1, 10);
		
		motor.config_kF(0, 0, 10);
		motor.config_kP(0, 0.2, 10);
		motor.config_kI(0, 0, 10);
		motor.config_kD(0, 0.005, 10);
		
		motor.configOpenloopRamp(0.1, 10);
		motor.configClosedloopRamp(0.1, 10);
	}
	
	public void updateSpeed(double speed) {
		motor.set(ControlMode.PercentOutput, speed);
	}
}
