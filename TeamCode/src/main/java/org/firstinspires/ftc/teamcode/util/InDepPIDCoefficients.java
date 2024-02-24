package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class InDepPIDCoefficients {

    //TODO: tune these constants
    public static final double MAX_VELOCITY = 0; // ticks per second either arm motor
    public static double VELOCITY_GAIN = 0; // position correction
    public static double VELOCITY_SPREAD_PROPORTION = 1; // nathan's favorite constant
                                                // should only be set within [0,1]
    public static double Kp = 0.0;
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
