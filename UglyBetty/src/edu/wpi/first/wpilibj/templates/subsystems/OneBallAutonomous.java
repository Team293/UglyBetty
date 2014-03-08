/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Ports;

/**
 *
 * @author Peter
 */
public class OneBallAutonomous {

    boolean hasFired = false, isTiming = false;
    final Gyro gyro;
    private static final double kStraight = 0.085, kAlign = 0.1;
    double startDriveTime = 0, stopTime = 3.1, alignTime = 1, driveSpeed = -0.82;
    Timer autoTimer;

    public OneBallAutonomous() {
        gyro = new Gyro(Ports.gyro);
        autoTimer = new Timer();
    }

    public void init() {
        autoTimer.start();
        Cage.release();
        gyro.reset();
        ShooterRack.finishedShooting();
        hasFired = false;
        isTiming = false;
    }

    public void run() {
//make sure you have a ball
        double blobCount = SmartDashboard.getNumber("blobCount", 0);
        DriveTrain.rangeUltrasonics();
        ShooterRack.run();
        Vision.setServo(0.65);
        SmartDashboard.putBoolean("hasFired", hasFired);
        SmartDashboard.putBoolean("possessing", Feeder.possessing());
        if (!Feeder.ballLimit2.get() && !hasFired) {
            SmartDashboard.putString("debugging", "looking for ball...");
            Feeder.triggerEnabled();
            Feeder.feed();
        } else {
            //mark start of driving/shooting
            if (!isTiming) {
                isTiming = true;
                startDriveTime = autoTimer.get();
            }
            Feeder.stop();
            //shooting loop 
            //t is time for driving
            double t = autoTimer.get() - startDriveTime;
            SmartDashboard.putNumber("time", t);
            if (blobCount == 2 && !hasFired && t >= stopTime) {
                SmartDashboard.putString("debugging", "starting to fire");
                hasFired = true;
                ShooterRack.startShooting();
                Feeder.triggerDisabled();
                Feeder.feed();
            } else if (autoTimer.get() > 9) {
                SmartDashboard.putString("debugging", "starting to fire");
                hasFired = true;
                ShooterRack.startShooting();
                Feeder.triggerDisabled();
                Feeder.feed();
            }
            //driving loop
            if (t < stopTime && !ShooterRack.isShooting()) {
                if (t < stopTime - alignTime) {
                    SmartDashboard.putString("debugging", "moving forward");
                    driveStraight(driveSpeed);
                } else {
                    SmartDashboard.putString("debugging", "aligning");
                    align();
                }
                Feeder.triggerEnabled();
                Feeder.stop();
            } else if (ShooterRack.isShooting()) {
                SmartDashboard.putString("debugging", "firing!!");
                DriveTrain.stop();
                Feeder.feed();
                Feeder.triggerDisabled();
                if (!Feeder.possessing()) {
                    SmartDashboard.putString("debugging", "resuming");
                    ShooterRack.finishedShooting();
                }
            } else {
                SmartDashboard.putString("debugging", "stopped");
                DriveTrain.stop();
            }
        }
    }

    public void align() {
        double angle = gyro.getAngle();
        //calculate motor output
        SmartDashboard.putNumber("gyro", angle);
        double turnOutput = kAlign * angle;
        if (turnOutput > 1) {
            turnOutput = 1;
        }
        if (turnOutput > 1) {
            turnOutput = 1;
        }
        DriveTrain.tankDrive(-turnOutput, turnOutput);
    }

    public void driveStraight(double speed) {
        //read the gyro
        double angle = gyro.getAngle();
        //calculate motor output
        SmartDashboard.putNumber("gyro", angle);
        double rightMotorOutput = speed - kStraight * angle;
        double leftMotorOutput = speed + kStraight * angle;
        if (rightMotorOutput > 1) {
            rightMotorOutput = 1;
        }
        if (leftMotorOutput > 1) {
            leftMotorOutput = 1;
        }
        if (rightMotorOutput < -1) {
            rightMotorOutput = -1;
        }
        if (leftMotorOutput < -1) {
            leftMotorOutput = -1;
        }
        //set motor output
        DriveTrain.tankDrive(-leftMotorOutput, -rightMotorOutput);
    }

}
