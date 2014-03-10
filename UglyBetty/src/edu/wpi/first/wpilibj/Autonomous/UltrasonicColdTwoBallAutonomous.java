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
public class UltrasonicColdTwoBallAutonomous {

    boolean hasFired = false, isTiming = false, firingFirst = false;
    final Gyro gyro;
    private static final double kStraight = 0.082,
            searchTime = 3,
            driveSpeedForward = -0.72,
            driveSpeedReverse = 0.64;

    double commandStartTime = 0;
    Timer autoTimer;

    public UltrasonicColdTwoBallAutonomous() {
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

    public void runColdGoal() {
        //feed 1
        Feeder.triggerEnabled();
        while (!Feeder.ballLimit2.get()) {
            Feeder.feed();
            Feeder.triggerEnabled();
            SmartDashboard.putString("debug..", "feeding");
        }
        markTime();
        Feeder.stop();
        Feeder.triggerEnabled();

        //move forward 1
        while (!DriveTrain.isAtShootingDistance()) {
            SmartDashboard.putString("debug..", "driving forward 1");
            Feeder.triggerEnabled();
            driveStraight(driveSpeedForward);
            ShooterRack.run();
            if (!Feeder.ballLimit2.get()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
        }
        Feeder.triggerEnabled();

        //shoot
        while (Feeder.possessing()) {
            SmartDashboard.putString("debug..", "shooting");
            ShooterRack.run();
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        Feeder.triggerEnabled();
        ShooterRack.stop();

        //back up && feed
        while (!Feeder.possessing()) {
            SmartDashboard.putString("debug..", "back up till feed");
            if (autoTimer.get() - commandStartTime < searchTime) {
                driveStraight(driveSpeedReverse);
            }
        }
        DriveTrain.stop();
        Feeder.stop();

        //move forward 2
        ShooterRack.setToShootingRPM();
        while (!DriveTrain.isAtShootingDistance()) {
            SmartDashboard.putString("debug..", "move forward 2");
            driveStraight(driveSpeedForward);
            ShooterRack.run();
            if (!Feeder.ballLimit2.get()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
        }
        DriveTrain.stop();
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

    public void markTime() {
        commandStartTime = autoTimer.get();
    }

}
