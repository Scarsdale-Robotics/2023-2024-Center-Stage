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
@TeleOp(name = "Arm PID Tuner")
public class PIDArmTuner extends LinearOpMode {
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

        // begin tuning sequence
        while (opModeIsActive()) {
//            double theta = Math.PI / 2;
//            drive.driveWithMotorPowers(
//                    0.25 * Math.sin(theta + Math.PI / 4), // fL
//                    0.25 * Math.sin(theta - Math.PI / 4), // fR
//                    0.25 * Math.sin(theta - Math.PI / 4), // bL
//                    0.25 * Math.sin(theta + Math.PI / 4)  // bR
//                    );

            // forward 1000√2 ticks
            inDep.raiseByEncoder(SpeedCoefficients.getArmSpeed(), ticks);
            while (opModeIsActive() && !gamepad1.triangle);
            // backward 1000√2 ticks
            inDep.raiseByEncoder(SpeedCoefficients.getArmSpeed(), -ticks);
            while (opModeIsActive() && !gamepad1.triangle);
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
