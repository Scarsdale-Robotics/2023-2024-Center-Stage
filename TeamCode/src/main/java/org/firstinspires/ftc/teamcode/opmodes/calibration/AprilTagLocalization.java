package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@Autonomous(name="April Tag Localization")
@Config
public class AprilTagLocalization extends LinearOpMode {
    public static int ID = 9;
    public static double aOffsetTarget = -25;
    public static double aSlowFactor = 0.001;
    public static double aSpeedCoef = 0.1;
    public static double aSpeedLimit = 0.1;
    public static double aThreshold = 0.001;
    public static boolean useFrontCamera = true;
    public static double xOffsetTarget = 0;
    public static double xSlowFactor = 0.001;
    public static double xSpeedCoef = -0.1;
    public static double xSpeedLimit = 0.5;
    public static double xThreshold = 0.02;
    public static double yOffsetTarget = 11;
    public static double ySlowFactor = 0.001;
    public static double ySpeedCoef = -0.1;
    public static double ySpeedLimit = 0.5;
    public static double yThreshold = 0.02;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        CVSubsystem cv = robot.getCV();
        cv.disablePropProcessor();
        cv.switchCamera(useFrontCamera ? cv.cameraName2 : cv.cameraName1);
        waitForStart();
        CVSubsystem.LocalizationState prev = new CVSubsystem.LocalizationState(false, 0, 0, 0);
        while (opModeIsActive() && !prev.unchanged) {
            prev = cv.faceTag(ID, xOffsetTarget, yOffsetTarget, aOffsetTarget, prev, xThreshold, yThreshold, aThreshold, xSpeedCoef, ySpeedCoef, aSpeedCoef, xSlowFactor, ySlowFactor, aSlowFactor, xSpeedLimit, ySpeedLimit, aSpeedLimit);
//            telemetry.addData("locX", cv.getPosition(0, 0)[0]);
//            telemetry.addData("locY", cv.getPosition(0, 0)[1]);
//            telemetry.addData("locRot", cv.getPosition(0, 0)[2]);
            telemetry.update();
        }
    }
}
