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

    boolean hasFired = false, isTiming = false, firingFirst = false;
    final Gyro gyro;
    private static final double kStraight = 0.085, kAlign = 0.089;
    double startDriveTime = 0,
            stopTime1 = 2.4,
            alignTime = 1,
            stopTime2 = 2.8,
            searchTime = 2.6,
            driveSpeed1 = -0.65,
            quickBack1 = 0.85,
            driveSpeed2 = 0.45;
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
        Feeder.triggerEnabled();
        while (!Feeder.ballLimit2.get()) {
            Feeder.feed();
            Feeder.triggerEnabled();
            SmartDashboard.putString("debug..", "feeding");
        }
        commandStartTime = autoTimer.get();
        Feeder.stop();
        Feeder.triggerEnabled();

        //move forward 1
        while (autoTimer.get() - commandStartTime < stopTime1) {
            SmartDashboard.putString("debug..", "driving forward 1");
            Feeder.triggerEnabled();
            driveStraight(driveSpeed1);
            ShooterRack.run();
            if (!Feeder.ballLimit2.get()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
        }
        commandStartTime = autoTimer.get();
        Feeder.triggerEnabled();

        //align to straight
        while (autoTimer.get() - commandStartTime < alignTime) {
            SmartDashboard.putString("debug..", "aligning");
            align();
            ShooterRack.run();
        }
        commandStartTime = autoTimer.get();

        //shoot
        while (Feeder.possessing()) {
            SmartDashboard.putString("debug..", "shooting");
            ShooterRack.run();
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        commandStartTime = autoTimer.get();
        Feeder.triggerEnabled();
        ShooterRack.stop();

        //back up && feed
        while (!Feeder.possessing()) {
            SmartDashboard.putString("debug..", "back up till feed");
            if (autoTimer.get() - commandStartTime < searchTime) {
                driveStraight(driveSpeed2);
            }
        }
        DriveTrain.stop();
        Feeder.stop();
        commandStartTime = autoTimer.get();

        //move forward 2
        while (autoTimer.get() - commandStartTime < stopTime2) {
            SmartDashboard.putString("debug..", "move forward 2");
            driveStraight(driveSpeed1);
            if (!Feeder.ballLimit2.get()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
        }
        commandStartTime = autoTimer.get();
        DriveTrain.stop();

        //align to straight 2
        while (autoTimer.get() - commandStartTime < alignTime) {
            SmartDashboard.putString("debug..", "aligning 2");
            align();
            ShooterRack.run();
        }

        //shoot
        while (Feeder.possessing()) {
            ShooterRack.run();
            SmartDashboard.putString("debug..", "shoot 2");
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        Feeder.triggerEnabled();
        ShooterRack.stop();
    }

    public void run3() {
        double blobCount = SmartDashboard.getNumber("blobCount", 0);
        commandStartTime = autoTimer.get();
        if (blobCount == 1 && (autoTimer.get() - commandStartTime) < 5) {//if we are not starting in a hot goal
            if (firingFirst == true) {
                firingFirst = false;
            }
            while (!Feeder.ballLimit2.get() || !Feeder.ballLimit.get()) {
                Feeder.feed();
                ShooterRack.run();
            }
            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < stopTime2) {
                driveStraight(driveSpeed1);
            }
        }
        if (blobCount == 2 && (autoTimer.get() - commandStartTime) < 5) {//we are starting in hot goal
            firingFirst = true;
            ShooterRack.run();
            while (!Feeder.ballLimit2.get() || !Feeder.ballLimit.get()) {
                Feeder.feed();
            }
            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < stopTime1) {
                driveStraight(driveSpeed1);
            }
            commandStartTime = autoTimer.get();
            while (Feeder.possessing()) {
                Feeder.triggerDisabled();
                Feeder.feed();
            }
            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < stopTime1) {
                driveFast(quickBack1);
                Feeder.feed();
            }
            while (!Feeder.ballLimit2.get() || !Feeder.ballLimit.get()) {
                Feeder.feed();
            }
            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < 2) {
                driveFast(quickBack1);
            }
            while (Feeder.possessing()) {
                Feeder.triggerDisabled();
                Feeder.feed();
            }
        }

        if (blobCount == 1 && (autoTimer.get() - commandStartTime) >= 5 && firingFirst == true) {//we have hopefully fired twice.
//this is a problem if we didn't finish shooting the second ball within 5 seconds
            //probably should make a variable to see if we sucessfully shot two balls or not.
            while (Feeder.possessing()) {
                Feeder.triggerDisabled();
                Feeder.feed();
            }
        }
        if (blobCount == 2 && (autoTimer.get() - commandStartTime) >= 5 && firingFirst == false) {//time to fire off the two balls
            while (Feeder.possessing()) {
                Feeder.triggerDisabled();
                Feeder.feed();
            }

            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < stopTime1) {
                driveFast(quickBack1);
                Feeder.feed();
            }
            while (!Feeder.ballLimit2.get() || !Feeder.ballLimit.get()) {
                Feeder.feed();
            }
            commandStartTime = autoTimer.get();
            while (autoTimer.get() - commandStartTime < 2) {
                driveFast(quickBack1);
            }
            while (Feeder.possessing()) {
                Feeder.triggerDisabled();
                Feeder.feed();
            }
            commandStartTime = autoTimer.get();
        } else {
//camera not working  PANIC!! or run the nonhotgoal two ball autonomous?????
        }

    }
    
    public void run4(){
        
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

    public void driveFast(double speed) {
        DriveTrain.tankDrive(speed, speed);

    }

}
