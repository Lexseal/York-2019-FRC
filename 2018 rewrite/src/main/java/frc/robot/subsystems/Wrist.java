/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Macro.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class Wrist extends Thread {
	int[] port;
	int freq;
	VictorSPX[] motor = new VictorSPX[2];
    GeneralPID wristController;
    Encoder encoder;
    double curAngle, desAngle;
    double curOmega;
    double lastTime;
    double lastDesAng;
    boolean actionComplete = false;
    DigitalInput wristLanding;
	
	public Wrist(int _encoderPortA, int _encoderPortB, int switchPort, int[] _port, int _freq) {
		port = _port;
        freq = _freq;
        wristController = new GeneralPID(3, 100, 0.6);
        wristController.setICap(20);
        wristLanding = new DigitalInput(switchPort);
		
		for (int i = 0; i < port.length; i++) {
			motor[i] = new VictorSPX(port[i]);
			motor[i].configNominalOutputForward(0, 10);
			motor[i].configNominalOutputReverse(0, 10);
			motor[i].configPeakOutputForward(1, 10);
            motor[i].configPeakOutputReverse(-1, 10);

            motor[i].setNeutralMode(NeutralMode.Brake);
		    motor[i].configOpenloopRamp(0.2, 10);
		    motor[i].configClosedloopRamp(0.2, 10);
        }
        
        encoder = new Encoder(_encoderPortA, _encoderPortB, false, Encoder.EncodingType.k4X);
    }
    
    public void updateConstants(double _kP, double _kI, double _kD) {
        wristController.setPIDConstants(_kP, _kI, _kD);
    }
	
	public void updateSpeed(double _speed) {
		motor[0].set(ControlMode.PercentOutput, _speed);
		motor[1].set(ControlMode.PercentOutput, _speed);
    }
    
    public double getWristAngle() {
        return encoder.get()/1024.0*360/117;
    }

    public double getCurSpeed() {
		return encoder.getRate()/1024.0*360/117;
    }
    
    public boolean isFinished() {
        return actionComplete;
    }

    public void setDesAng(double _desAng) {
        desAngle = _desAng;
        double bias = 0;
        if (lastDesAng != desAngle) {
            double diff = desAngle-lastDesAng;
            lastDesAng = desAngle;
            if (diff > 0) {
                bias = 0;
            } else if (diff < 0) {
                bias = -0;
            }
        }
        desAngle += bias;
    }

    public void zeroSensor() {
        encoder.reset();
    }
	
	public void run() {
        lastTime = System.currentTimeMillis()/1000;

		while (true) {
			try {
				Thread.sleep(1000/freq);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
            
            if (!wristLanding.get()) {
                zeroSensor();
                //System.out.println("landed");
            }

            double curTime = System.currentTimeMillis()/1000;
            double deltaTime = curTime-lastTime;
            double lastTime = curTime;

            curAngle = getWristAngle();
            double error = desAngle-curAngle;
            double speed = getCurSpeed();
            //SmartDashboard.putString(SDLMotor, "en "+curAngle);
            
            double output = wristController.getOutput(error, speed);
            //SmartDashboard.putString(SDLMotor, ""+error);
            if (error < 0 && curAngle < 75) {
                output /= 10;
            }

            if (output > 80) {
                output = 80;
            } else if (output < -80) {
                output = -80;
            }
            updateSpeed(output/100);

            if (Math.abs(error) < 3) {
                actionComplete = true;
            } else {
                actionComplete = false;
            }
		}
	}
}
