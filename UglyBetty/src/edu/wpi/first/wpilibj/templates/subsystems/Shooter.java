/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.SpikeEncoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Peter
 */
public class Shooter {

    SpikeEncoder encoder;
    Talon talon;
    static int ID, tolerance = 30;
    private double setpoint, output, rpm, error, kP;
    int id;

    Shooter(int talonPort, int encAPort, int encBPort) {
        talon = new Talon(talonPort);
        encoder = new SpikeEncoder(encAPort, encBPort, SpikeEncoder.BLACK);
        setpoint = 0;
        output = 0;
        rpm = 0;
        error = 0;
        kP = -0.000014;
        this.id = ++ID;
    }

    public void init() {
        encoder.start();
    }

    public void stop() {
        talon.set(0);
        output = 0;
    }

    public void readSetpoint() {
        this.setpoint = SmartDashboard.getNumber("setpoint" + id, 0);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void run() {
        rpm = -encoder.getRPM();
        error = setpoint - rpm;

        kP = SmartDashboard.getNumber("kP" + id, kP);

        output += error * kP;

        SmartDashboard.putNumber("rpm" + id, rpm);
        SmartDashboard.putNumber("setpoint" + id, setpoint);
        SmartDashboard.putNumber("error" + id, error);
        SmartDashboard.putNumber("output" + id, output);
        if (output < -1) {
            output = -1;
        }
        if (output > 1) {
            output = 1;
        }
        talon.set(-output);
    }

    public boolean atRPM() {
        return Math.abs(error) < tolerance;
    }
}
