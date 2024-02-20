package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@TeleOp(name = "Velocity PID Tuner")
public class PIDVelocityTuner extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private static DriveSubsystem drive;
    private static InDepSubsystem inDep;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());


    @Override
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        inDep = robot.getInDep();
        drive = robot.getDrive();
        runtime.reset();
        inDep.close();

        waitForStart();

        MovementSequence forward = new MovementSequenceBuilder()
                .forward(60)
                .build();


        // begin tuning sequence
        while (opModeIsActive()) {
            // drive with 30 ticks/sec for 2 secs maybe
            runtime.reset();
            drive.leftFrontController.resetIntegral();
            drive.leftBackController.resetIntegral();
            drive.rightFrontController.resetIntegral();
            drive.rightBackController.resetIntegral();
            while (opModeIsActive() && ((runtime.milliseconds() < 2000) || !gamepad1.triangle)) {
                drive.updateMotorVelocities(1000, 1000, 1000, 1000);
                telemetry.update();
            }
//            drive.stopController();

            // drive with -30 ticks/sec for 2 secs maybe
            runtime.reset();
            drive.leftFrontController.resetIntegral();
            drive.leftBackController.resetIntegral();
            drive.rightFrontController.resetIntegral();
            drive.rightBackController.resetIntegral();
            while (opModeIsActive() && ((runtime.milliseconds() < 2000) || !gamepad1.triangle)) {
                drive.updateMotorVelocities(-1000, -1000, -1000, -1000);
                telemetry.update();
            }
//            drive.stopController();
        }

        drive.stopController();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }

}