package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class InDepPIDCoefficients {
    public static double Kp = 0.0025;
    public static double Ki = 0.0;
    public static double Kd = 0.00005;
    public static double errorTolerance_p = 10.1;
    public static double errorTolerance_v = 0.25;

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
}
