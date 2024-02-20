package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivePIDCoefficients {

    public static double MAX_VELOCITY = 2000; // wheels ticks per second on ground
    public static double POWER_GAIN = 0.11; // for velocity control in wheels
    public static double TURN_VELOCITY_GAIN = 65; // for position control in drive movements
    public static double VELOCITY_GAIN = 10; // for position control in drive movements
    public static double VELOCITY_SPREAD_PROPORTION = 0.8; // should only fall within [0, 1], for drive velocity curve

    public static double drive_Kd = 0.01;
    public static double drive_Ki = 0.0;
    public static double drive_Kp = 0.0075;

    public static double errorTolerance_degrees = 1.0;
    public static double errorTolerance_p = 10.5;
    public static double errorTolerance_v = 1;

    public static double turn_Kd = 0.001;
    public static double turn_Ki = 0.1;
    public static double turn_Kp = 0.1;

    public static double velocity_Kd = 0.0000002;
    public static double velocity_Ki = 0.0000001;
    public static double velocity_Kp = 0.002;


    public static double getDriveP() {
        return drive_Kp;
    }

    public static double getDriveI() {
        return drive_Ki;
    }

    public static double getDriveD() {
        return drive_Kd;
    }

    public static double getVelocityP() {
        return velocity_Kp;
    }

    public static double getVelocityI() {
        return velocity_Ki;
    }

    public static double getVelocityD() {
        return velocity_Kd;
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
