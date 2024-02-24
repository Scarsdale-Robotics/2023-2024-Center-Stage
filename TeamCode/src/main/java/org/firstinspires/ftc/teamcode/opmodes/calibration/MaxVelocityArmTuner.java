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
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@TeleOp(name = "Max Velocity Arm Tuner")
public class MaxVelocityArmTuner extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot hardwareRobot;
    private static InDepSubsystem inDep;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());

    public static double ticks = 2000;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.leftArm,
                hardwareRobot.rightArm,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                this,
                telemetry
        );
        inDep.close();
        runtime.reset();

        waitForStart();


        double L_max = 0, R_max = 0;
        // begin tuning sequence
        while (opModeIsActive()) {

            double sum = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(sum) > 0.5)
                inDep.setArmPowers(sum);
            else
                inDep.stopMotors();

            double L_velocity = inDep.getLeftArmVelocity();
            double R_velocity = inDep.getRightArmVelocity();

            L_max = Math.max(Math.abs(L_max), Math.abs(L_velocity));
            R_max = Math.max(Math.abs(R_max), Math.abs(R_velocity));

            telemetry.addData("Left Arm Velocity", L_velocity);
            telemetry.addData("Right Arm Velocity", R_velocity);

            telemetry.addData("Left Arm Max Velocity", L_max);
            telemetry.addData("Right Arm Max Velocity", R_max);

            telemetry.update();

        }

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
