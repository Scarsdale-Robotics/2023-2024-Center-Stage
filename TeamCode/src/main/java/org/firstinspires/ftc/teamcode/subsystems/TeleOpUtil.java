package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class TeleOpUtil {
    private final DriveSubsystem drive;
    private final InDepSubsystem inDep;
    private final CVSubsystem cv;
    private final boolean isRedTeam;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final AlignmentUtility align;
    public TeleOpUtil(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedTeam, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                opMode
        );
        inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                opMode,
                telemetry
        );
        cv = new CVSubsystem(
                robot.camera,
                drive
        );
        align = new AlignmentUtility(drive, cv)
        this.isRedTeam = isRedTeam;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private void runMoveModeControl() {
        if (gamepad2.dpad_up) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        } else if (gamepad2.dpad_down) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        }
    }

    private void runDriveControl() {
        drive.driveRobotCentric(
                -Math.signum(gamepad1.left_stick_x) * SpeedCoefficients.getStrafeSpeed(),
                Math.signum(gamepad1.left_stick_y) * SpeedCoefficients.getForwardSpeed(),
                -gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed()
        );
    }

    /**
     * PRIMARY MOTION CONTROL METHOD
     */
    private void runMotionControl() {
        runMoveModeControl();
        runDriveControl();
    }

    private void runClawToggleControl() {
        if (gamepad1.y) {
            if (inDep.getIsOpen())
                inDep.close();
            else
                inDep.open();
        }
    }

    private void runArmFlexControl() {
        inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());
    }

    private void runArmRigidControl() {
        if (gamepad2.right_bumper)
            inDep.raiseArm();
        else if (gamepad2.left_bumper)
            inDep.lowerArm();
    }

    private void runResetArmControl() {
        if (gamepad1.a || gamepad2.a) {
            inDep.resetArmEncoder();
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
    }

    /**
     * PRIMARY ARM CLAW CONTROL METHOD
     */
    private void runArmClawControl() {
        runClawToggleControl();
        runArmFlexControl();
        runArmRigidControl();
        runResetArmControl();
    }

    private void

    public void tick() {
        runMotionControl();
        runArmClawControl();

    }
}
