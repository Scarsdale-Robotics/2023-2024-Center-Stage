package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleFunction;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double minIntegral;
    private double maxIntegral;
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

        this.minIntegral = -1.0;
        this.maxIntegral = 1.0;

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
//        integralSum = integralSum < minIntegral ? minIntegral : Math.min(maxIntegral, integralSum);

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//        output = Math.min(1.0, output);
//        output = Math.max(-1.0, output);

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