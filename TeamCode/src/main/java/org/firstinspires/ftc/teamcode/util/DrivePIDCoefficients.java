package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivePIDCoefficients {
    public static double Kp = 0.02;
    public static double Ki = 0.15;
    public static double Kd = 0.0006;
    public static double errorTolerance_p = 7.5;
    public static double errorTolerance_v = 0.05;
    public static double errorTolerance_degrees = 2.5;

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

    public static double getErrorTolerance_v() {
        return errorTolerance_v;
    }

    public static double getErrorTolerance_degrees() {
        return errorTolerance_degrees;
    }
}
