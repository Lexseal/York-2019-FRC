/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import static frc.robot.Macro.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
public class FeedForwardController {
  private double gain, kP;
  private double cap;

  public FeedForwardController(double _gain, double _kP) {
    cap = maxPID;
    gain = _gain;
    kP = _kP;
  }

  public void setConstants(double _gain, double _kP) {
    gain = _gain;
    kP = _kP;
  }

  public void setCap(double _cap) {
    cap = _cap;
  }

  public double getOutput(double command, double error) {
    double outputGain = gain*command;

    double outputError = error*kP;

    double output = outputGain+outputError;
    if (output > cap) {
      output = cap;
    } else if (output < -cap) {
      output = -cap;
    }
    //SmartDashboard.putString(SDAngErr, ""+((int)((outputGain)*100))/100.0);//+" "+((int)((outputI)*100))/100.0+ " "+((int)((outputD)*100))/100.0);
    return output;
  }
}
