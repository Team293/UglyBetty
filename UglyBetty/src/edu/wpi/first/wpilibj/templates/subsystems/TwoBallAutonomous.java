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
    private static final double kStraight = 0.085;
    double startDriveTime = 0, stopTime1 = 3.4, stopTime2 = 6, driveSpeed1 = -0.85, driveSpeed2 = 0.85;
    int runCount =1;
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
    }

    public void run() {
        DriveTrain.rangeUltrasonics();
        ShooterRack.run();
        //feed ball
        if (!Feeder.possessing() && !hasFired) {
            SmartDashboard.putString("debugging", "looking for ball...");
            Feeder.triggerEnabled();
            Feeder.feed();
        } else {
            //stop feeder on possession
            Feeder.stop();
            
        }z
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
