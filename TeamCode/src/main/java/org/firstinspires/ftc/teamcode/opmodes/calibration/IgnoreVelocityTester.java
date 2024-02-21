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
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.opencv.core.Mat;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@TeleOp(name = "Ignore Velocity Boolean Test")
public class IgnoreVelocityTester extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot hardwareRobot;
    private static DriveSubsystem drive;
    private static InDepSubsystem inDep;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());

    public static double displacement = 5;


    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        inDep = robot.getInDep();
        drive = robot.getDrive();
        runtime.reset();
        inDep.close();

        waitForStart();

        MovementSequence forward = new MovementSequenceBuilder()
                .forward(displacement)
                .forward(displacement, false, true)
                .forwardRight(displacement, displacement, false, true)
                .forward(displacement)
                .build();

        MovementSequence backward = new MovementSequenceBuilder()
                .backward(displacement, false, true)
                .backwardLeft(displacement, displacement, false, true)
                .backward(displacement)
                .backward(displacement)
                .build();

        // begin tuning sequence
        while (opModeIsActive()) {

            // forward 1000√2 ticks
//            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), 3000, 3000, Math.PI/2);
            drive.followMovementSequence(forward);
            while (opModeIsActive() && !gamepad1.triangle);
            // backward 1000√2 ticks
//            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), 3000, 3000, -Math.PI/2);
            drive.followMovementSequence(backward);
            while (opModeIsActive() && !gamepad1.triangle);
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
