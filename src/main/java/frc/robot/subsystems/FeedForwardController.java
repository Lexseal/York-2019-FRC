/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Macro.*;

/**
 * feed forward controller
 */
public class FeedForwardController {
    private double gain, kP;
    private double cap;

    /**
     * init the controller with just a gain
     * 
     * @param _gain
     */
    public FeedForwardController(double _gain) {
        cap = maxPID;
        gain = _gain;
    }

    /**
     * init the controller with a gain and a kP
     * 
     * @param _gain
     * @param _kP
     */
    public FeedForwardController(double _gain, double _kP) {
        cap = maxPID;
        gain = _gain;
        kP = _kP;
    }

    /**
     * update the constants on the go
     * 
     * @param _gain
     * @param _kP
     */
    public void setConstants(double _gain, double _kP) {
        gain = _gain;
        kP = _kP;
    }

    /**
     * update the cap on the go
     * 
     * @param _cap
     */
    public void setCap(double _cap) {
        cap = _cap;
    }

    /**
     * get the output without error correction
     * 
     * @param command
     * @return
     */
    public double getOutput(double command) {
        double outputGain = gain * command;

        double output = outputGain;
        if (output > cap) {
            output = cap;
        } else if (output < -cap) {
            output = -cap;
        }
        return output;
    }

    /**
     * get the output with error correction
     * 
     * @param command
     * @param error
     * @return
     */
    public double getOutput(double command, double error) {
        double outputGain = getOutput(command);

        double outputError = error * kP;

        double output = outputGain + outputError;
        if (output > cap) {
            output = cap;
        } else if (output < -cap) {
            output = -cap;
        }
        return output;
    }
}
