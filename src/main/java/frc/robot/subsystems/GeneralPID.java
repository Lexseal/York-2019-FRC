/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Macro.*;

/**
 * Provide your constants and get the PID results
 */
public class GeneralPID {
    private double PIDCap; // cap on the total amount of output
    private double kP, kI, kD; // PID constants
    private double ICap, lastI; // cap on "I" specifically
    private double lastTime, curTime; // state variables

    /**
     * construct the controller with PID constants
     * 
     * @param _kP
     * @param _kI
     * @param _kD
     */
    public GeneralPID(double _kP, double _kI, double _kD) {
        PIDCap = maxPID;
        kP = _kP;
        kI = _kI;
        kD = _kD;
        lastI = 0;

        curTime = System.currentTimeMillis() / 1000.0;
        lastTime = curTime;
    }

    /**
     * construct the controller with PID constants and a cap on "I"
     * 
     * @param _kP
     * @param _kI
     * @param _kD
     * @param ICap
     */
    public GeneralPID(double _kP, double _kI, double _kD, double ICap) {
        this(_kP, _kI, _kD);
        this.ICap = ICap;
    }

    /**
     * updating the PID constants on the go
     * 
     * @param _kP
     * @param _kI
     * @param _kD
     */
    public void setPIDConstants(double _kP, double _kI, double _kD) {
        this.kP = _kP;
        this.kI = _kI;
        this.kD = _kD;
    }

    /**
     * update the cap on I on the go
     * 
     * @param _ICap
     */
    public void setICap(double _ICap) {
        ICap = _ICap;
    }

    /**
     * zero the I value in PID calculation
     */
    public void zeroI() {
        lastI = 0;
    }

    /**
     * get the output by providing the current error and the rate of change of error
     * 
     * @param error
     * @param dError
     * @return output of the PID controller
     */
    public double getOutput(double error, double dError) {
        double outputP = kP * error;

        curTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = curTime - lastTime;
        if (deltaTime > 0.1) {
            deltaTime = 0.1;
        }
        lastTime = curTime;
        lastI += kI * error * deltaTime;
        if (lastI > ICap) {
            lastI = ICap;
        } else if (lastI < -ICap) {
            lastI = -ICap;
        }
        double outputI = lastI;

        double outputD = -kD * dError;

        double output = outputP + outputI + outputD;
        if (output > PIDCap) {
            output = PIDCap;
        } else if (output < -PIDCap) {
            output = -PIDCap;
        }
        return output;
    }
}
