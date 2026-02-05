package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double kP, kI, kD, kF;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public double calculate(double target, double current) {

        double error = target - current;

        double dt = timer.seconds();
        timer.reset();

        // Prevent divide-by-zero or crazy derivative spikes
        if (dt <= 0) dt = 0.001;

        // Integral term
        integralSum += error * dt;

        // Derivative term
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void setCoefficients(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }
}

