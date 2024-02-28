package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivePIDCoefficients {

    public static final double MAX_THEORETICAL_VELOCITY = 2250; // wheels ticks per second on ground
    public static double MAX_RUN_VELOCITY = 0.9 * MAX_THEORETICAL_VELOCITY; //TODO: retune pid if this causes robot to go short
    public static double MAX_ADJUSTED_VELOCITY = MAX_RUN_VELOCITY; // adjusted to scale with displacement
    public static double VELOCITY_GAIN = 50; // for position control in drive movements
    public static double VELOCITY_SPREAD_PROPORTION = 0.4; // should only fall within [0, 1], for drive velocity curve

    public static double TURN_VELOCITY_GAIN = 25; // for heading control in drive movements
    public static double TURN_POSITION_GAIN = 0.0005; // for heading control in drive movements

    public static double drive_Kd = 0.0124;
    public static double drive_Ki = 0.0;
    public static double drive_Kp = 0.05;

    public static final double MAX_THEORETICAL_ANGULAR_VELOCITY = 230;// degrees per second on ground
    public static double MAX_RUN_ANGULAR_VELOCITY = 0.9 * MAX_THEORETICAL_ANGULAR_VELOCITY;
    public static double MAX_ADJUSTED_ANGULAR_VELOCITY = MAX_RUN_ANGULAR_VELOCITY; // adjusted to scale with displacement
    public static double ANGULAR_VELOCITY_GAIN = 10; // for heading control in turn movements
    public static double ANGULAR_VELOCITY_SPREAD_PROPORTION = 1; // should only fall within [0, 1]

    public static double turn_Kd = 0.05;
    public static double turn_Ki = 0.0;
    public static double turn_Kp = 0.5;

    public static double errorTolerance_degrees = 0.0;
    public static double errorTolerance_p = 0.0;


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

    public static double getErrorTolerance_degrees() {
        return errorTolerance_degrees;
    }
}
