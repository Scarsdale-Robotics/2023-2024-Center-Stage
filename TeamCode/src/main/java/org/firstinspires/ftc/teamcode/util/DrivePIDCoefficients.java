package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivePIDCoefficients {
    public static double drive_Kp = 0.0075;
    public static double drive_Ki = 0.05;
    public static double drive_Kd = 0.0001;
    public static double turn_Kp = 0.1;
    public static double turn_Ki = 0.1;
    public static double turn_Kd = 0.001;
    public static double errorTolerance_p = 15.5;
    public static double errorTolerance_v = 0.05;
    public static double errorTolerance_degrees = 1.0;

    public static double getDriveP() {
        return drive_Kp;
    }

    public static double getDriveI() {
        return drive_Ki;
    }

    public static double getDriveD() {
        return drive_Kd;
    }

    public static double getTurnP() {
        return turn_Kp;
    }

    public static double getTurnI() {
        return turn_Ki;
    }

    public static double getTurnD() {
        return turn_Kd;
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
