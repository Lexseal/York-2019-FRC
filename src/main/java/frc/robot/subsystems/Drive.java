package frc.robot.subsystems;

import static frc.robot.Macro.*;
import edu.wpi.first.wpilibj.DriverStation;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * a class that handles robot driving very specific to the robot this class is
 * very patchy
 */
public class Drive extends Thread {
    int[] leftMotorNum;
    int[] rightMotorNum; // Motor CAN ports.

    /*
     * In a 4-motor config, 0 and 1 are left motors while 2 and 3 are right motors.
     * In a 6-motor config, 0, 1, and 2 are left while 3, 4, and 5 are right.
     */
    TalonSRX[] motor = new TalonSRX[MAX];

    VictorSPX[] littleMotor = new VictorSPX[2]; // two small motors on the extended leg

    ADIS16448_IMU imu;
    DriverStation station = DriverStation.getInstance();

    /* driving is handled by two feed forward controllers */
    FeedForwardController turnController;
    FeedForwardController speedController;

    /* physics */
    double curAng, desAng, curOmega, desOmega, lastOmega, curAlpha;
    double desPos, curSpeed, desSpeed, curPos;
    double[] desDisp = new double[2], desVelocity = new double[2];
    double[] curDisp = new double[2], curVelocity = new double[2];
    double throttle = 0, lastThrottle = 0;
    double desX, x, desY, y; // robot position
    double lastTime, deltaTime;
    double freq;
    double I = 0; // integral term
    double lastTurn = 0; // register the turn command at last cycle to determine if breaking is needed
                         // this cycle
    double liftHeight = 0;
    double liftRestrictionMultiplier = 0;
    double restrictionMultiplier = 1;
    boolean isFollowMode = false;
    boolean isRecording = false;
    boolean littleWheelsActive = false; // really really patchy code... sorry!

    /**
     * provide the CAN ID of the left and right motors as well as the frequency the
     * drive class runs
     * 
     * @param _left
     * @param _right
     * @param _freq
     * @param _imu
     */
    public Drive(int _left[], int _right[], double _freq, ADIS16448_IMU _imu) {
        leftMotorNum = _left;
        rightMotorNum = _right;
        for (int i = 0; i < _left.length; i++) {
            motor[i] = new TalonSRX(_left[i]); // Init left motors using CAN.
            motor[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            motor[i].setInverted(false);
            motor[i].setSensorPhase(true);
        }
        for (int i = 0; i < _right.length; i++) {
            motor[i + _left.length] = new TalonSRX(_right[i]); // Init right motors.
            motor[i + _left.length].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            motor[i + _left.length].setInverted(true);
            motor[i + _left.length].setSensorPhase(true);
        }

        for (int i = 0; i < _left.length + _right.length; i++) {
            motor[i].setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10); // modify the encoder refresh
                                                                                          // rate

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
        imu.calibrate(); // Init gyro.

        turnController = new FeedForwardController(0.15, 0.5);
        speedController = new FeedForwardController(driveGain1, drivekP1);

        freq = _freq;

        /* patch for the small motors on the leg */
        littleMotor[0] = new VictorSPX(7);
        littleMotor[0].setInverted(true);
        littleMotor[1] = new VictorSPX(12);
        littleMotor[1].setInverted(false);
        /* end of patch */
    }

    /**
     * get velocity
     * 
     * @return the left and the right wheel speed
     */
    public double[] getcurVelocity() {
        double[] encoderReadOut = {
                (double) motor[0].getSelectedSensorVelocity(0) * 10.0 / ticksPerRev * wheelCircumfrence,
                (double) motor[2].getSelectedSensorVelocity(0) * 10.0 / ticksPerRev * wheelCircumfrence };
        return encoderReadOut;
    }

    /**
     * get displacement
     * 
     * @return the left the the right wheel distance travelled
     */
    private double[] getcurDisp() {
        double[] encoderReadOut = { (double) motor[0].getSelectedSensorPosition(0) / ticksPerRev * wheelCircumfrence,
                (double) motor[2].getSelectedSensorPosition(0) / ticksPerRev * wheelCircumfrence };
        return encoderReadOut;
    }

    /**
     * update robot's position on the field unused
     */
    /*private void updateCoordinate() {
        double dc = curSpeed * deltaTime;
        x += Math.sin(curAng * 3.1415926 / 180) * dc;
        y += Math.cos(curAng * 3.1415926 / 180) * dc;
    }*/

    /**
     * activate the little wheels
     */
    public void activateLittleWheels() {
        littleWheelsActive = true;
    }

    /**
     * deactivate the little wheels to save power
     */
    public void deActivateLittleWheels() {
        littleWheelsActive = false;
    }

    private void updateMotor(double leftOutput, double rightOutput, ControlMode mode) {
        // SmartDashboard.putString(SDLMotor, ""+leftOutput);
        // SmartDashboard.putString(SDRMotor, ""+rightOutput);
        motor[0].set(mode, leftOutput);
        motor[1].set(ControlMode.Follower, motor[0].getDeviceID());
        motor[2].set(mode, rightOutput);
        motor[3].set(ControlMode.Follower, motor[2].getDeviceID());

        if (littleWheelsActive) {
            littleMotor[0].set(mode, leftOutput * 1);
            littleMotor[1].set(mode, rightOutput * 1);
        }
    }

    /**
     * update the robot velocity with the desired speed and the turn speed
     * 
     * @param _speed
     * @param _turn
     */
    public void updateVelocity(double _speed, double _turn) {
        desSpeed = _speed * maxSpeed;
        desOmega = _turn * maxOmega;
        lastTurn = _turn;
        followFinished();
    }

    /**
     * update the robot's position with the desired angle of travel and distance not
     * in use!
     * 
     * @param _angle
     * @param _displacement
     */
    public void updateDisplacement(double _angle, double _displacement) {
        // achieve angle
        desAng = curAng + _angle;
        System.out.println("ang and disp" + _angle + " " + _displacement + " desiAng " + desAng);
        double startTime = System.currentTimeMillis() / 1000.0;
        while (Math.abs(desAng - imu.getAngle()) > 1 || (System.currentTimeMillis() / 1000.0 - startTime) < 1000
                || (station.isAutonomous() && station.isEnabled())) {
        }

        // achieve displacement
        /*
         * curPos =
         * (motor[0].getSelectedSensorPosition(0)+motor[2].getSelectedSensorPosition(0))
         * /2; curPos = curPos/wheelMultiplier*wheelCircumfrence;
         */
        // desPos = curPos+_displacement;
        startTime = System.currentTimeMillis() / 1000.0;
        while (station.isAutonomous() && station.isEnabled()) {
            /*
             * curPos =
             * (motor[0].getSelectedSensorPosition(0)+motor[2].getSelectedSensorPosition(0))
             * /2; curPos = curPos/wheelMultiplier*wheelCircumfrence;
             */
            // double error = desPos-curPos;

            // if (Math.abs(error) < 0.05 /*||
            // (System.currentTimeMillis()/1000.0-startTime)>_displacement*2000*/) {
            // updateVelocity(0, 0);
            // break;
            // } else {
            // double power = Math.abs(0.25-Math.pow((error-_displacement/2),
            // 4)/(0.625*Math.pow(_displacement, 4)));
            // double power = Math.abs(0.6-Math.pow((error-_displacement/2),
            // 2)/(0.71*Math.pow(_displacement, 2)));
            // double iniP = 0.2, finP = 0.35+Math.abs(_displacement)*0.1;
            // double dter = (16/Math.pow(_displacement, 2)+2);
            // double a = (finP-iniP)*dter;
            // double b = +Math.abs(_displacement)/2;
            // double c = iniP-a/(2+Math.pow(b, 2));
            // double power = a/(2+Math.pow((Math.abs(error)-b), 2))+c;

            // if (error > 0) {
            // System.out.println(power+" "+curPos+" "+error);
            // updateVelocity(power, 0);
            // } else {
            // System.out.println(power+" "+curPos+" "+error);
            // updateVelocity(-power, 0);
            // }
            // }
        }
    }

    /**
     * go to position x, y not in use!
     * 
     * @param _x
     * @param _y
     */
    public void updatePosition(double _x, double _y) {
        double dx = _x - x;
        double dy = _y - y;
        double theta = -((Math.atan2(dy, dx)) * (180 / 3.1416) - 90);
        double correctAng = (curAng) % 360;
        double dTheta = theta - correctAng;
        double displacement = Math.hypot(dx, dy);
        System.out.println("dTheta" + dTheta + " " + displacement);
        updateDisplacement(dTheta, displacement);
    }

    /**
     * for autonomous follow mode use
     */
    public void followFinished() {
        isFollowMode = false;
    }

    /**
     * motion profile following in autonomous
     * 
     * @param speedL
     * @param speedR
     * @param leftPos
     * @param rightPos
     */
    public void follow(double speedL, double speedR, double leftPos, double rightPos) {
        isFollowMode = true;
        desVelocity[0] = speedL;
        desVelocity[1] = speedR;
        desDisp[0] = leftPos;
        desDisp[1] = rightPos;

        // desX = _x;
        // desY = _y;
    }

    /**
     * set drive constants on the fly
     * 
     * @param _turnGain
     * @param _turnkP
     * @param _driveGain
     * @param _drivekP
     */
    public void setConstants(double _turnGain, double _turnkP, double _driveGain, double _drivekP) {
        turnController.setConstants(_turnGain, _turnkP);
        speedController.setConstants(_driveGain, _drivekP);
    }

    /**
     * zero the imu
     * 
     * @return true if the sequence has been completed
     */
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

    /**
     * patch 1 to prevent the 2018 robot from tipping
     */
    public void restrictedAcc() {
        restrictionMultiplier = 0.3;
    }

    /**
     * patch 2 to prevent the 2018 robot from tipping
     */
    public void normalAcc() {
        restrictionMultiplier = 1;
    }

    /**
     * disable the drive during recording as we are pushing the robot around
     * 
     * @param value whether we are recording the path
     */
    public void setRecordingStat(boolean value) {
        isRecording = value;
    }

    /**
     * thread function this gets called periodically to update the speed of the
     * robot
     * 
     * implementation specific to each robot for reference only
     */
    @Override
    public void run() {
        desAng = curAng;

        x = 0;
        y = 0;

        lastTime = System.currentTimeMillis() / 1000.0; // Init lastTime for integral calculation.
        lastOmega = imu.getRate();

        /* to achieve the right frequency the function is run */
        while (true) {
            try {
                Thread.sleep((long) (1000 / freq));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double curTime = System.currentTimeMillis() / 1000.0;
            deltaTime = curTime - lastTime; // Get deltaTime
            lastTime = curTime;

            curAng = imu.getAngle();
            curOmega = imu.getRate(); // Get omega
            curAlpha = (curOmega - lastOmega) / deltaTime;
            // SmartDashboard.putString(SDcurAng, ""+curAng);

            curDisp = getcurDisp();
            curPos = (curDisp[0] + curDisp[1]) / 2;
            curVelocity = getcurVelocity();
            curSpeed = (curVelocity[0] + curVelocity[1]) / 2.0;

            double angError = desAng - curAng; // Get angle error
            angError = angError % 360;
            if (angError > 180) {
                angError -= 360;
            } else if (angError < -180) {
                angError += 360;
            }

            double omegaError = desOmega - curOmega;
            double speedError = desSpeed - curSpeed;

            double outputThrottle = speedController.getOutput(desSpeed, speedError);
            outputThrottle /= 100; // divide the output by 100

            double outputTurn = turnController.getOutput(desOmega, omegaError);
            outputTurn /= 100;

            // update state variable
            lastThrottle = throttle;
            lastOmega = curOmega;

            /* disable the motor if we are recording the path */
            if (isRecording) {
                updateMotor(0, 0, ControlMode.PercentOutput);
            } else {
                updateMotor(outputThrottle + outputTurn, outputThrottle - outputTurn, ControlMode.PercentOutput);
            }
        }
    }

    public void setLiftHeight(double curPos2) {
        liftHeight = curPos2;
    }
}
