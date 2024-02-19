package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@Autonomous()
public class AprilTagLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        CVSubsystem cv = robot.getCV();
        cv.disablePropProcessor();
        cv.switchCamera(cv.cameraName1);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("locX", cv.getPosition(0, 0)[0]);
            telemetry.addData("locY", cv.getPosition(0, 0)[1]);
            telemetry.addData("locRot", cv.getPosition(0, 0)[2]);
            cv.goToPosition(0, 0, new double[]{1.5, 1.5, 0}, false);
            telemetry.update();
        }
    }
}
