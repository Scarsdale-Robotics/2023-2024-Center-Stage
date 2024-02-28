package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class InDepPIDCoefficients {

    public static final double MAX_THEORETICAL_VELOCITY = 2250;// ticks per second either arm motor
    public static final double MAX_RUN_VELOCITY = 0.9 * MAX_THEORETICAL_VELOCITY;
    public static double MAX_ADJUSTED_VELOCITY = MAX_RUN_VELOCITY; // adjusted to scale with displacement
    public static double VELOCITY_GAIN = 10; // position correction
    public static double VELOCITY_SPREAD_PROPORTION = 1; // should only be set within [0,1]

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static double errorTolerance_p = 0.0;


    public static double getKp() {
        return Kp;
    }

    public static double getKi() {
        return Ki;
    }

    public static double getKd() {
        return Kd;
    }

    public static double getErrorTolerance_p() {
        return errorTolerance_p;
    }

}
