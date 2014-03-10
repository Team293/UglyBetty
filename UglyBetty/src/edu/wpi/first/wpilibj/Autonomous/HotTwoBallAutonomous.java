/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.Autonomous;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Ports;
import edu.wpi.first.wpilibj.templates.subsystems.Cage;
import edu.wpi.first.wpilibj.templates.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.templates.subsystems.Feeder;
import edu.wpi.first.wpilibj.templates.subsystems.ShooterRack;

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
    }

    public void run() {
        //check blob count
        blobCount = SmartDashboard.getNumber("blobCount", 0);

        //enable trigger
        Feeder.triggerEnabled();

        //feed
        while (!Feeder.ballLimit2.get() && !Feeder.ballLimit.get()) {
            Feeder.feed();
            SmartDashboard.putString("debug..", "feeding");
        }
        markTime();

        //drive foreward for stopTime1 at driveSpeed1
        while (autoTimer.get() - commandStartTime < stopTime1) {
            driveStraight(driveSpeed1);
        }
        markTime();

        //turn and then shoot first ball
        while (Feeder.possessing()) {
            ShooterRack.run();
            //if left goal is hot, turn left
            if (blobCount == 2 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnLeft);
            } //if left goal is NOT hot, turn right
            else if (blobCount == 1 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnRight);
            } else {
                SmartDashboard.putString("debug..", "shooting");
                Feeder.triggerDisabled();
                Feeder.feed();
            }
            markTime();
        }
        ShooterRack.stop();

        //turn back to straight
        while (autoTimer.get() - commandStartTime < turnTime) {
            if (blobCount == 1) {
                turn(turnLeft);
            } else if (blobCount == 2) {
                turn(turnRight);
            }
        }
        markTime();

        //back up to next ball
        //why stopTime1-1?????????
        while (autoTimer.get() - commandStartTime < stopTime1 - 1) {
            driveStraight(-driveSpeed1);
            Feeder.triggerEnabled();
        }

        //feed second ball
        while (!Feeder.possessing()) {
            driveStraight(-driveSpeed1 / 2);
            Feeder.feed();
        }
        markTime();

        //drive forward
        while (autoTimer.get() - commandStartTime < stopTime2) {
            ShooterRack.run();
            driveStraight(driveSpeed2);
        }

        //turn and shoot
        while (!Feeder.possessing()) {
            //if left goal was not hot the first time, turn left this time
            if (blobCount == 1 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnLeft);
            } //if left goal was hot the first time, turn right this time
            else if (blobCount == 2 && autoTimer.get() - commandStartTime < turnTime) {
                turn(turnRight);
            } else {
                Feeder.triggerDisabled();
                Feeder.feed();
                ShooterRack.run();
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
