/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import static frc.robot.Macro.*;

/**
 * Add your docs here.
 */
public class GeneralPID {
  private double PIDCap;
  private double kP, kI, kD;
  private double ICap, lastI;
  private double lastTime, curTime;

  public GeneralPID(double _kP, double _kI, double _kD) {
    PIDCap = maxPID;
    kP = _kP;
    kI = _kI;
    kD = _kD;
    this.ICap = ICap;
    lastI = 0;

    curTime = System.currentTimeMillis()/1000.0;
    lastTime = curTime;
  }

  public GeneralPID(double _kP, double _kI, double _kD, double ICap) {
    this(_kP, _kI, _kD);
    this.ICap = ICap;
  }

  public void setPIDConstants(double _kP, double _kI, double _kD) {
    this.kP = _kP;
    this.kI = _kI;
    this.kD = _kD;
  }

  public void setICap(double _ICap) {
    ICap = _ICap;
  }

  public void zeroI() {
    lastI = 0;
  }

  public double getOutput(double error, double dError) {
    double outputP = kP*error;

    curTime = System.currentTimeMillis()/1000.0;
    double deltaTime = curTime-lastTime;
    if (deltaTime > 0.1) {
      deltaTime = 0.1;
    }
    lastTime = curTime;
    lastI += kI*error*deltaTime;
    if (lastI > ICap) {
      lastI = ICap;
    } else if (lastI < -ICap) {
      lastI = -ICap;
    }
    double outputI = lastI;

    double outputD = -kD*dError;

    double output = outputP+outputI+outputD;
    if (output > PIDCap) {
      output = PIDCap;
    } else if (output < -PIDCap) {
      output = -PIDCap;
    }
    return output;
  }
}
