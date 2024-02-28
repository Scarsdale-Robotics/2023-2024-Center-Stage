package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@TeleOp(name = "Max Angular Velocity Tuner")
public class MaxAngularVelocityTuner extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot hardwareRobot;
    private static DriveSubsystem drive;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());


    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                this,
                false,
                telemetry
        );
        runtime.reset();

        waitForStart();

        double max = -1;
        // begin tuning sequence
        while (opModeIsActive()) {

            double power = -gamepad1.right_stick_x;
            if (Math.abs(power) > 0.5)
                drive.driveRobotCentric(0, 0, power);
            else
                drive.stopController();

            double velocity = drive.getAngularVelocity();

            max = Math.max(Math.abs(max), Math.abs(velocity));

            telemetry.addData("Angular Velocity", velocity);

            telemetry.addData("Max Angular Velocity", max);

            telemetry.update();

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
