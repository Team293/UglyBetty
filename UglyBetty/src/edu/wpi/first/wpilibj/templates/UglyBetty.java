/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.subsystems.Cage;
import edu.wpi.first.wpilibj.templates.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.templates.subsystems.Feeder;
import edu.wpi.first.wpilibj.templates.subsystems.ShooterRack;
import edu.wpi.first.wpilibj.templates.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class UglyBetty extends IterativeRobot {

    boolean hasFired;
    DriverStationLCD LCD = DriverStationLCD.getInstance();
    final Gyro gyro = new Gyro(Ports.gyro);
    private static final double kStraight = 0.083;
    Timer t = new Timer();
    Timer autoTimer = new Timer();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        ShooterRack.init();
        Feeder.triggerEnabled();
        ShooterRack.setToShootingRPM();
    }

    public void teleopInit() {
        Cage.release();
    }

    public void autonomousInit() {
        autoTimer.start();
        Cage.release();
        gyro.reset();
        hasFired = false;
        isTiming = false;
        t.reset();
        t.stop();
    }

    /**
     * This function is called periodically during autonomous
     */
    boolean isTiming = false;
    double stopTime = 3;

    public void autonomousPeriodic() {
        //make sure you have a ball
        double blobCount = SmartDashboard.getNumber("blobCount", 0);
        DriveTrain.rangeUltrasonics();
        ShooterRack.run();
        Vision.setServo(0.65);
        SmartDashboard.putBoolean("hasFired", hasFired);
        SmartDashboard.putBoolean("possessing", Feeder.possessing());
        if (!Feeder.possessing() && !hasFired) {
            SmartDashboard.putString("debugging", "looking for ball...");
            Feeder.triggerEnabled();
            Feeder.feed();
        } else {
            if (!isTiming) {
                isTiming = true;
                t.start();
            }
            Feeder.stop();
            //shooting loop 
            SmartDashboard.putNumber("time", t.get());
            if (blobCount == 2 && !hasFired && t.get() >= stopTime) {
                SmartDashboard.putString("debugging", "starting to fire");
                hasFired = true;
                ShooterRack.startShooting();
                Feeder.triggerDisabled();
                Feeder.feed();
            } else if (autoTimer.get() > 8) {
                SmartDashboard.putString("debugging", "starting to fire");
                hasFired = true;
                ShooterRack.startShooting();
                Feeder.triggerDisabled();
                Feeder.feed();
            }
//            //driving loop
            if (t.get() < stopTime && !ShooterRack.isShooting()) {
                SmartDashboard.putString("debugging", "moving forward");
                if (t.get() < stopTime - 1) {
                    driveStraight(-0.75);
                } else {
                    driveStraight(-0.2);
                }
                Feeder.triggerEnabled();
                Feeder.stop();
            } else if (ShooterRack.isShooting()) {
                SmartDashboard.putString("debugging", "firing!!");
                t.stop();
                DriveTrain.stop();
                Feeder.feed();
                Feeder.triggerDisabled();
                if (!Feeder.possessing()) {
                    SmartDashboard.putString("debugging", "resuming");
                    ShooterRack.finishedShooting();
                    t.start();
                }
            } else {
                SmartDashboard.putString("debugging", "stopped");
                DriveTrain.stop();
            }
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //LEDs.indicateSituation();
        OperatorInterface.controlDriveTrain();
        OperatorInterface.controlShooter();
        OperatorInterface.controlFeeder();
        OperatorInterface.controlAutoAlign();
        OperatorInterface.controlCamera();
        DriveTrain.rangeUltrasonics();
        LCD.println(DriverStationLCD.Line.kUser1, 1, "" + DriveTrain.getLeftDistance());
        LCD.println(DriverStationLCD.Line.kUser2, 1, "" + DriveTrain.getRightDistance());

        LCD.updateLCD();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
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
