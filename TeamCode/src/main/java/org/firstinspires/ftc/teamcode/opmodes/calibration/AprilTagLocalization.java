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
    public static double speedFactor = 2;
    public static double slowFactor = 1;
    public static double angleFactor = 0.005;
    public static double xyOffsetThreshold = 0.1;
    public static double angleOffsetThreshold = 5;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        CVSubsystem cv = robot.getCV();
        cv.disablePropProcessor();
        cv.switchCamera(cv.cameraName1);
        waitForStart();
        while (opModeIsActive() && !cv.goToPosition(0, 0, new double[]{1.5, 1.5, 0}, speedFactor, slowFactor, angleFactor, xyOffsetThreshold, angleOffsetThreshold)) {
            telemetry.addData("locX", cv.getPosition(0, 0)[0]);
            telemetry.addData("locY", cv.getPosition(0, 0)[1]);
            telemetry.addData("locRot", cv.getPosition(0, 0)[2]);
            telemetry.update();
        }
    }
}
