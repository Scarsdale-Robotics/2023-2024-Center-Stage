package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleFunction;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double minIntegral;
    private double maxIntegral;
    private double minOutput;
    private double maxOutput;
    private double setPoint;
    private double integralSum;
    private double lastError;
    private double errorTolerance;
    private ElapsedTime timer;

    public PIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0);
    }

    public PIDController(double Kp, double Ki, double Kd, double setPoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.minIntegral = Double.MIN_VALUE;
        this.maxIntegral = Double.MAX_VALUE;

        this.minOutput = Double.MIN_VALUE;
        this.maxOutput = Double.MAX_VALUE;

        this.setPoint = setPoint;

        this.integralSum = 0;
        this.lastError = 0;

        this.errorTolerance = 0;

        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the control value, u(t).
     * @param error The current error of the process variable.
     * @return the value produced by u(t).
     */
    public double updateError(double error) {
        return update(setPoint-error);
    }

    /**
     * Calculates the control value, u(t).
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double update(double pv) {
        // Calculate the error
        double error = setPoint - pv;

        // Calculate derivative
        double derivative = (error - lastError) / timer.seconds(); // secant line slope

        // Calculate integral
        integralSum += error * timer.seconds(); // riemann sum rectangle
        integralSum = Math.min(maxIntegral, integralSum);
        integralSum = Math.max(minIntegral, integralSum);

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        output = Math.min(maxOutput, output);
        output = Math.max(minOutput, output);

        // Update stuff for derivative
        lastError = error;
        timer.reset();

        return output; // Return multiplier
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void resetIntegral() {
        this.integralSum = 0;
    }

    public void setIntegralBounds(double lowerBound, double higherBound) {
        minIntegral = lowerBound;
        maxIntegral = higherBound;
    }

    public void setOutputBounds(double lowerBound, double higherBound) {
        minOutput = lowerBound;
        maxOutput = higherBound;
    }

    public double getSetPoint() {
        return this.setPoint;
    }

    public boolean atSetPoint(double pv) {
        return Math.abs(pv-setPoint) < errorTolerance;
    }

    public double getAbsoluteDiff(double pv) {
        return Math.abs(pv-setPoint);
    }

    public void setErrorTolerance(double errorTolerance) {
        this.errorTolerance = errorTolerance;
    }

}