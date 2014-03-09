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
public class HotTwoBallAutonomous {

    boolean hasFired = false, isTiming = false, firingFirst = false;
    final Gyro gyro;
    private static final double kStraight = 0.082, kAlign = 0.089;
    double startDriveTime = 0,
            alignTime = 0.5,
            stopTime1 = 2.35,
            stopTime2 = 2.70,
            searchTime = 2.90,
            quickBack1 = 0.85,
            driveSpeed1 = -0.69,
            driveSpeed2 = 0.64,
            driveSpeed3 = -0.74,
            turnLeft = 20,
            blobCount = 0,
            turnTime = 0.75,
            turnRight = (-turnLeft);

    double commandStartTime = 0;
    int runCount = 1;
    Timer autoTimer;

    public HotTwoBallAutonomous() {
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
        Feeder.triggerEnabled();

        blobCount = SmartDashboard.getNumber("blobCount", 0);
    }

    public void run() {
        ShooterRack.run();
        Feeder.triggerEnabled();
        while (!Feeder.ballLimit2.get() && !Feeder.ballLimit.get()) {//feed
            Feeder.feed();
            SmartDashboard.putString("debug..", "feeding");
        }
        markTime();
        while (autoTimer.get() - commandStartTime < stopTime1) {//drive foreward
            driveStraight(driveSpeed1);
        }
        markTime();
        while (Feeder.possessing()) {//turn and then shoot first ball
            if (blobCount == 2 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnLeft);
            }
            if (blobCount == 1 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnRight);
            } else {
                SmartDashboard.putString("debug..", "shooting");
                ShooterRack.run();
                Feeder.triggerDisabled();
                Feeder.feed();
            }
            markTime();
        }
        while (autoTimer.get() - commandStartTime < turnTime) {
            if (blobCount == 1) {
                turn(turnLeft);
            }
            if (blobCount == 2) {
                turn(turnRight);
            }
        }

        markTime();
        while (autoTimer.get() - commandStartTime < stopTime1 - 1) {
            driveStraight(-driveSpeed1);
            Feeder.triggerEnabled();
            Feeder.feed();
            ShooterRack.stop();
        }
        while (!Feeder.possessing()) {
            driveStraight(-driveSpeed1 / 2);
            Feeder.feed();
        }
        markTime();
        while (autoTimer.get() - commandStartTime < stopTime2) {
            ShooterRack.run();
            driveStraight(driveSpeed2);
        }
        while (!Feeder.possessing()) {
            if (blobCount == 1 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnLeft);
            }
            if (blobCount == 2 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnRight);
            }
            Feeder.triggerDisabled();
            Feeder.feed();
            ShooterRack.run();
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
        if (turnOutput < -1) {
            turnOutput = -1;
        }
        DriveTrain.tankDrive(-turnOutput, turnOutput);
    }

    public void turn(double turnAngle) {
        double angle = gyro.getAngle();
        //calculate motor output
        angle = angle - turnAngle;
        SmartDashboard.putNumber("gyro", angle);
        double turnOutput = kAlign * angle;
        if (turnOutput > 1) {
            turnOutput = 1;
        }
        if (turnOutput < -1) {
            turnOutput = -1;
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

    public void driveFast(double speed) {
        DriveTrain.tankDrive(speed, speed);
    }

    public void markTime() {
        commandStartTime = autoTimer.get();
    }

}
