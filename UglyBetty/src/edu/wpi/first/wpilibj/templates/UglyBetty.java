/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.subsystems.Cage;
import edu.wpi.first.wpilibj.templates.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.templates.subsystems.Feeder;
import edu.wpi.first.wpilibj.templates.subsystems.OneBallAutonomous;
import edu.wpi.first.wpilibj.templates.subsystems.ShooterRack;
import edu.wpi.first.wpilibj.templates.subsystems.ColdTwoBallAutonomous;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class UglyBetty extends IterativeRobot {

    DriverStationLCD LCD = DriverStationLCD.getInstance();
    OneBallAutonomous oneBallAuto;
    ColdTwoBallAutonomous twoBallAuto;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        SmartDashboard.putNumber("lowRPM", 0);
        SmartDashboard.putNumber("middleRPM", 0);
        SmartDashboard.putNumber("highRPM", 0);
//        if (OperatorInterface.oneBalAutonomous()) {
//            oneBallAuto = new OneBallAutonomous();
//        } else {
        twoBallAuto = new ColdTwoBallAutonomous();
        //}
        ShooterRack.init();
        Feeder.triggerEnabled();
        ShooterRack.setToShootingRPM();
    }

    public void teleopInit() {
        Cage.release();
    }

    public void autonomousInit() {
//        if (OperatorInterface.oneBalAutonomous()) {
//            oneBallAuto.init();
//        } else {
        twoBallAuto.init();
        //}
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
//        if (OperatorInterface.oneBalAutonomous()) {
//            oneBallAuto.run();
//        } else {
        twoBallAuto.runColdGoal();
        //}
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //LEDs.indicateSituation();
        OperatorInterface.controlDriveTrain();
        OperatorInterface.controlShooter();
        OperatorInterface.controlFeeder();
        //OperatorInterface.controlAutoAlign();
        OperatorInterface.controlCamera();
        //DriveTrain.rangeUltrasonics();
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
}
