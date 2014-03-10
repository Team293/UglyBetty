/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.Autonomous;

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
public class UltrasonicOneBallAutonomous extends Auto {

    public UltrasonicOneBallAutonomous() {
        super();
    }

    public void init() {
        super.init();
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
            driveStraight(driveSpeedForward);
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
}
