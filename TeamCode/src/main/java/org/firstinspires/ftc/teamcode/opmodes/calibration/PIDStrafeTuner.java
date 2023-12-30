package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@Autonomous(name = "Drivetrain Strafe PID Tuner")
public class PIDStrafeTuner extends LinearOpMode {
    private static double Kp = 0.01;
    private static double Ki = 0;
    private static double Kd = 0;
    private static double errorTolerance_p = 5.0;
    private static double errorTolerance_v = 0.05;

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
                telemetry
        );
        runtime.reset();

        waitForStart();

        // begin tuning sequence
        while (opModeIsActive()) {
            // right 1000√2 ticks
            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), -500, 500, 0);
            while (opModeIsActive() && !gamepad1.triangle);
            // left 1000√2 ticks
            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), 500, -500, Math.PI);
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

    /*
    public void driveByAngularEncoder(double driveSpeed, double leftTicks, double rightTicks, double theta) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            throw new RuntimeException("driveByAngularEncoder(): Tried to run two drive actions at once");
        }

        // begin action
        double L = leftTicks, R = rightTicks;

        PIDController L_PID = new PIDController(Kp, Ki, Kd, L);
        PIDController R_PID = new PIDController(Kp, Ki, Kd, R);

        while ( // checking condition
                opModeIsActive() &&
                        Math.abs(L-leftBack.getCurrentPosition()) > errorTolerance_p &&
                        Math.abs(R-rightBack.getCurrentPosition()) > errorTolerance_p &&
                        Math.abs(drive.getLeftWheelVelocity()) > errorTolerance_v &&
                        Math.abs(getRightWheelVelocity()) > errorTolerance_v
        ) {
            // getting L and R
            double L_p = leftBack.getCurrentPosition();
            double R_p = rightBack.getCurrentPosition();

            if (telemetry != null) {
                telemetry.addData("L setpoint",L);
                telemetry.addData("L position",L_p);
                telemetry.addData("R setpoint",R);
                telemetry.addData("R position",R_p);
                telemetry.addData("L power",getLeftWheelVelocity());
                telemetry.addData("R power",getRightWheelVelocity());
                telemetry.update();
            }

            double L_K = L_PID.update(L_p);
            double R_K = R_PID.update(R_p);

            double L_v = driveSpeed * Math.sin(theta - Math.PI / 4) * L_K; // for bL and fR
            double R_v = driveSpeed * Math.sin(theta + Math.PI / 4) * R_K; // for bR and fL

            // setting the powers of the motors
            driveWithMotorPowers(
                    R_v, // fL
                    L_v, // fR
                    L_v, // bL
                    R_v  // bR
            );

            isBusy = true;
        }

        // brake
        controller.stop();
        isBusy = false;
    }*/
}