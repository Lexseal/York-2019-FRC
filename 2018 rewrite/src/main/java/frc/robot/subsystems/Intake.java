package frc.robot.subsystems;

//import static org.usfirst.frc.team5171.robot.Macro.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends Thread {
	int port;
	int freq;
	VictorSPX motor;
	double speed;
	boolean newCommand = false;
	
	public Intake(int _port, int _freq) {
		port = _port;
		freq = _freq;

		motor = new VictorSPX(port);
		motor.configNominalOutputForward(0, 10);
		motor.configNominalOutputReverse(0, 10);
		motor.configPeakOutputForward(1, 10);
		motor.configPeakOutputReverse(-1, 10);
	}
	
	public void updateSpeed(double _speed) {
        speed = _speed;
		motor.set(ControlMode.PercentOutput, _speed);
	}
	
	public void run() {
		while (true) {
			try {
				Thread.sleep(1000/freq);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			updateSpeed(speed);
		}
	}
}
