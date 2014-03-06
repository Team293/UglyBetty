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
public class TwoBallAutonomous {

    boolean hasFired = false, isTiming = false;
    final Gyro gyro;
    private static final double kStraight = 0.085, kAlign = 0.09;
    double startDriveTime = 0, stopTime1 = 2.4, alignTime = 1, stopTime2 = 3, driveSpeed1 = -0.85, driveSpeed2 = 0.85;
    double commandStartTime = 0;
    int runCount = 1;
    Timer autoTimer;

    public TwoBallAutonomous() {
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

    public void run2() {
        //feed 1
        while (!Feeder.ballLimit2.get()) {
            Feeder.feed();
        }
        commandStartTime = autoTimer.get();
        Feeder.stop();

        //move forward 1
        while (autoTimer.get() - commandStartTime < stopTime1) {
            driveStraight(driveSpeed1);
            ShooterRack.run();
        }
        commandStartTime = autoTimer.get();

        //align to straight
        while (autoTimer.get() - commandStartTime < alignTime) {
            align();
            ShooterRack.run();
        }
        commandStartTime = autoTimer.get();

        //shoot
        while (Feeder.possessing()) {
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        commandStartTime = autoTimer.get();
        Feeder.triggerEnabled();
        ShooterRack.stop();

        //back up && feed
        while (!Feeder.possessing()) {
            driveStraight(driveSpeed2);
        }
        DriveTrain.stop();
        Feeder.stop();
        commandStartTime = autoTimer.get();

        //move forward 2
        while (autoTimer.get() - commandStartTime < stopTime2) {
            driveStraight(driveSpeed1);
        }
        DriveTrain.stop();

        //shoot
        while (Feeder.possessing()) {
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        Feeder.triggerEnabled();
        ShooterRack.stop();
    }

    public void run1() {
        DriveTrain.rangeUltrasonics();
        ShooterRack.run();
        //feed ball
        if (!Feeder.possessing() && !hasFired && (runCount == 1 || runCount == 2)) {
            SmartDashboard.putString("debugging", "looking for ball...");
            Feeder.triggerEnabled();
            Feeder.feed();
        } else if (Feeder.possessing()) {
            //stop feeder on possession
            Feeder.stop();
            startDriveTime = autoTimer.get();
            if (!isTiming) {
                isTiming = true;

            }

            if (!hasFired &&) {

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
