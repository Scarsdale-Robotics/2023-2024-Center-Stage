package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double minIntegral;
    private double maxIntegral;
    private double setPoint;
    private double integralSum;
    private double lastError;
    private ElapsedTime timer;

    public PIDController(double Kp, double Ki, double Kd, double setPoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.minIntegral = -1.0;
        this.maxIntegral = 1.0;

        this.setPoint = setPoint;

        this.integralSum = 0;
        this.lastError = 0;

        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the control value, u(t).
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double update(double encoderPosition) {
        // Calculate the error
        double error = setPoint - encoderPosition;

        // Calculate derivative
        double derivative = (error - lastError) / timer.seconds(); // secant line slope

        // Calculate integral
        integralSum += error * timer.seconds(); // riemann sum rectangle
        integralSum = integralSum < minIntegral ? minIntegral : Math.min(maxIntegral, integralSum);

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Update stuff for derivative
        lastError = error;
        timer.reset();

        return output; // Return multiplier
    }
}