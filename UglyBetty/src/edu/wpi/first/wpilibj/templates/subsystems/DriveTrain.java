/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Ports;

/**
 *
 * @author Robotics Laptop B
 */
public class DriveTrain {

    public static final Victor leftMotor = new Victor(Ports.leftDrive);
    public static final Victor rightMotor = new Victor(Ports.rightDrive);
    public static RobotDrive drive = new RobotDrive(leftMotor,
            rightMotor);
    static final AnalogChannel rightUltrasonic = new AnalogChannel(Ports.rightUltrasonic);
    static final AnalogChannel leftUltrasonic = new AnalogChannel(Ports.leftUltrasonic);
    static final DigitalOutput ultrasonicSignal = new DigitalOutput(Ports.ultrasonicSignal);
    private static double rightDistance, leftDistance;
    private static int ping = 0;
    private static final int sampleSize = 10;
    private static final double[] val = new double[sampleSize];

    public static void tankDrive(double leftMotor, double rightMotor) {
        drive.tankDrive(leftMotor, -rightMotor);
    }

    public static void emptyFunction() {
    }

    public static void stop() {
        drive.tankDrive(0.01, 0.01);
    }

    public static void rangeUltrasonics() {
        //ping ultrasonic sensors
        ping++;
        if (ping % 5 == 0) {
            ultrasonicSignal.pulse(0.0001);
        }
        leftDistance = convertToDistance(leftUltrasonic.getAverageVoltage());
        rightDistance = convertToDistance(rightUltrasonic.getAverageVoltage());
        //sensor array stuff
        for (int i = 0; i < val.length - 2; i++) {
            val[i] = val[i + 2];
        }
        val[sampleSize - 2] = leftDistance;
        val[sampleSize - 1] = rightDistance;

        sort(val);

        double adjustedDistance = (val[6] + val[7] + val[8] + val[9]) / 4.0;
        SmartDashboard.putNumber("leftD", leftDistance);
        SmartDashboard.putNumber("rightD", rightDistance);
        SmartDashboard.putNumber("adjustedD", adjustedDistance);
    }

    public static double getLeftDistance() {
        return leftDistance;
    }

    public static double getRightDistance() {
        return rightDistance;
    }

    public static boolean isAligned() {
        double difference = leftDistance - rightDistance;
        double average = (leftDistance + rightDistance) / 2.0;
        SmartDashboard.putNumber("aligned", difference);
        SmartDashboard.putNumber("distanced", average);
        SmartDashboard.putBoolean("aligned", difference < 0.4);
        SmartDashboard.putBoolean("distanced", Math.abs(average - 12) < 1);
        return Math.abs(average - 12) < 1 && difference < 0.4;
    }

    public static void moveToDistance() {
        double difference = leftDistance - rightDistance;
        double average = (leftDistance + rightDistance) / 2.0;
        SmartDashboard.putNumber("average dstiance", average);
        if (difference < 5) {
            drive.tankDrive(0.4, 0.4);
        } else {
            drive.tankDrive(0, 0);
        }
    }

    public static double convertToDistance(double rawVoltage) {
        return (rawVoltage + 0.0056) / 0.1141;
    }

    public static void sort(double[] a) {
        while (!inOrder(a)) {
            for (int i = 0; i < a.length; i++) {
                //if in wrong order, flip
                if (a[i] > a[i + 1]) {
                    double temp = a[i + 1];
                    a[i + 1] = a[i];
                    a[i] = temp;
                }
            }
        }
    }

    public static boolean inOrder(double[] a) {
        for (int i = 0; i < a.length; i++) {
            if (a[i] > a[i + 1]) {
                return false;
            }
        }
        return true;
    }
}
