package frc.robot.subsystems;

import static frc.robot.Macro.*;

import edu.wpi.first.wpilibj.Joystick;

public class Controller extends Thread {
	int[] axisList;
	int[] buttonList;
	double[] axisInput = new double[MAX];
	double[] lastInput = new double[MAX];
	boolean[] buttonInput = new boolean[MAX];
	int POVInput;
	double deadBand, cutOff, freq, expo;
	Joystick stick;
	
	public Controller(int _joyPort, int _axisList[], int _buttonList[], double _deadBand, double _cutOff, double _freq, double _expo) {
		axisList = _axisList;
		for (int i = 0; i < axisList.length; i++) {
			axisInput[axisList[i]] = 0;
		}
		lastInput = axisInput;
		deadBand = _deadBand/100;
		cutOff = _cutOff/100;
		expo = _expo;
		
		buttonList = _buttonList;
		for (int i = 0; i < buttonList.length; i++) {
			buttonInput[buttonList[i]] = false;
		}

		POVInput = -1;
		
		freq = _freq;
		stick = new Joystick(_joyPort);
	}
	
	public double getAxis(int _port) {
		if (axisInput[_port] < 0) {
			return -Math.pow(Math.abs(axisInput[_port]), expo);
		}
		return Math.pow(Math.abs(axisInput[_port]), expo);
	}
	
	public boolean getButton(int _port) {
		return buttonInput[_port];
	}

	public int getPOV() {
		return POVInput;
	}
	
	public void run() {
		while(true) {
			try {
				sleep((long)(1000/freq));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			for (int i = 0; i < axisList.length; i++) {
				int curPort = axisList[i];
				axisInput[curPort] = stick.getRawAxis(curPort);
				if (Math.abs(axisInput[curPort]-lastInput[curPort])/(1/freq)<=deadBand) {
					axisInput[curPort] = lastInput[curPort];
				} else {
					axisInput[curPort] = (axisInput[curPort]+lastInput[curPort])/2;
				}
				if (Math.abs(axisInput[curPort]) < cutOff) {
					axisInput[curPort] = 0;
				}
				
				lastInput[i] = axisInput[i];
			}
			
			for (int i = 0; i < buttonList.length; i++) {
				int curPort = buttonList[i];
				buttonInput[curPort] = stick.getRawButton(curPort);
			}

			POVInput = stick.getPOV();
		}
	}
}
