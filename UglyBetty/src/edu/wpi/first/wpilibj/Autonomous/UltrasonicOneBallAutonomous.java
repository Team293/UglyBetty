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
public class UltrasonicOneBallAutonomous {

    boolean hasFired = false, isTiming = false;
    final Gyro gyro;
    private static final double kStraight = 0.085, kAlign = 0.1;
    double startDriveTime = 0, stopDistance = 7, alignTime = 1, driveSpeed = -0.82;
    Timer autoTimer;

    public UltrasonicOneBallAutonomous() {
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
        double blobCount = SmartDashboard.getNumber("blobCount", 0);
        ShooterRack.setToShootingRPM();
        DriveTrain.rangeUltrasonics();
        Feeder.triggerEnabled();
        //feed in ball
        while (!Feeder.possessing()) {
            Feeder.feed();
        }
        Feeder.stop();

        //move forward until at shooting Distance
        while (!DriveTrain.isAtShootingDistance()) {
            DriveTrain.rangeUltrasonics();
            driveStraight(driveSpeed);
            //make sure you have a ball
            if (!Feeder.possessing()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
            //spin up shooter wheels
            ShooterRack.run();
        }
        //FIRE!!!
        while (Feeder.possessing()) {
            ShooterRack.run();
            Feeder.triggerDisabled();
            Feeder.feed();
        }
        ShooterRack.stop();
        DriveTrain.stop();
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
