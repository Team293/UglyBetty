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
             turnLeft=20,
            blobCount=0,
            turnTime=0.75,
             turnRight=(-turnLeft);
    
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
        
        blobCount = SmartDashboard.getNumber("blobCount", 0);
    }

    public void runColdGoal() {
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
            driveStraight(driveSpeed3);
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
        ShooterRack.setToShootingRPM();
        while (autoTimer.get() - commandStartTime < stopTime2) {
            SmartDashboard.putString("debug..", "move forward 2");
            driveStraight(driveSpeed1);
            ShooterRack.run();
            if (!Feeder.ballLimit2.get()) {
                Feeder.feed();
            } else {
                Feeder.stop();
            }
        }
        commandStartTime = autoTimer.get();
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

    public void runHotGoal() {
        double blobCount = SmartDashboard.getNumber("blobCount", 0);
        commandStartTime = autoTimer.get();
        //change if to while and ==1 to !=2
        while (blobCount != 2 && (autoTimer.get() - commandStartTime) < 5) {//if we are not starting in a hot goal
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

    
    public void turningHotGoal()
    {   
      ShooterRack.run();
      Feeder.triggerEnabled();
        while (!Feeder.ballLimit2.get()&&!Feeder.ballLimit.get()) {//feed
            Feeder.feed();
            SmartDashboard.putString("debug..", "feeding");
        }
        markTime();
        while (autoTimer.get() - commandStartTime<stopTime1){//drive foreward
            driveStraight(driveSpeed1);
        }
        markTime();
        while(Feeder.possessing()){//turn and then shoot first ball
            if (blobCount==2&&autoTimer.get() - commandStartTime<turnTime){
                turn(turnLeft);
            }
            if(blobCount==1&&autoTimer.get() - commandStartTime<turnTime){
                turn(turnRight);
            }
            else{
            SmartDashboard.putString("debug..", "shooting");
            ShooterRack.run();
            Feeder.triggerDisabled();
            Feeder.feed();
             }
            markTime();
        }
              while(autoTimer.get() - commandStartTime<turnTime){
                if (blobCount==1){
                turn(turnLeft);
            }
                if(blobCount==2){
                turn(turnRight);
            }   
         }
   
        
            markTime();
        while(autoTimer.get() - commandStartTime<stopTime1-1){
       driveStraight(-driveSpeed1); 
       Feeder.triggerEnabled();
       Feeder.feed();
       ShooterRack.stop();
       }
        while(!Feeder.possessing()){
        driveStraight(-driveSpeed1/2);
        Feeder.feed();
        }
        markTime();
        while(autoTimer.get() - commandStartTime<stopTime2){
         ShooterRack.run(); 
         driveStraight(driveSpeed2);         
        }
        while(!Feeder.possessing()){
       if (blobCount==1&&autoTimer.get() - commandStartTime<turnTime){
                turn(turnLeft);
            }
            if(blobCount==2&&autoTimer.get() - commandStartTime<turnTime){
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
        angle=angle-turnAngle;
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
    public void markTime(){
         commandStartTime = autoTimer.get();
    }

}
