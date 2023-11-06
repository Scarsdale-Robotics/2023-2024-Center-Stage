package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class TeleOpUtil {
    public HardwareRobot robot;
    public DriveSubsystem drive;
    public InDepSubsystem inDep;
    public CVSubsystem cv;
    private final boolean isRedTeam;
    private boolean clawToggle = false;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public TeleOpUtil(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedTeam, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        robot = new HardwareRobot(hardwareMap);
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
        this.isRedTeam = isRedTeam;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private void runMoveModeControl() {
        if (gamepad1.dpad_up) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        } else if (gamepad1.dpad_down) {
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
        if (gamepad1.y && !clawToggle) {
            if (inDep.getIsOpen()) inDep.close();
            else inDep.open();
            clawToggle = true;
        }
        if (!gamepad1.y) clawToggle = false;
    }

    private void runArmFlexControl() {
        inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());
    }

    private void runArmRigidControl() {
//        if (gamepad1.right_bumper)
//            inDep.raiseArm();
//        else if (gamepad1.left_bumper)
//            inDep.lowerArm();
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

    /**
     * will likely be used in auto, not teleop
     * this method is for testing
     * controls are for testing as well
     */
    private void runAprilTagAlignmentControl() {
        if (gamepad1.dpad_right) cv.moveToAprilTag(1);  // feel free to adjust id from 1-6 inclusive
    }

    /**
     * temp method, will be moved to auto after testing
     * ignore that the controls are inconvenient--they are temporary for testing
     */
    private void teamPropLocationControl() {
        if (gamepad1.dpad_left) {
            int teamPropLocation = cv.getTeamPropLocation(isRedTeam);
            switch (teamPropLocation) {
                case 0:
                    // left location
                    drive.driveRobotCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
                    break;
                case 2:
                    // right location
                    drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
                    break;
                default:
                    // center location
                    drive.driveRobotCentric(0, 1 * SpeedCoefficients.getForwardSpeed(), 0);
                    break;
            }
        };
    }

    private void runAprilTagParallelAlignControl() {
        // check alignParallelWithAprilTag() for details
        if (gamepad1.b) cv.alignParallelWithAprilTag(isRedTeam ? 5 : 2);
    }

    /**
     * macro controls are wip, feel free to change as is comfortable
     * also don't question the one-line methods... have to keep a consistent style...
     */
    private void runPixelAlignmentControl() {
        if (gamepad1.b) cv.moveToPixel();
    }

    public void tick() {
        // TODO: check if wentao and tutu have pushed the tape detection pipeline that takes into account isRedTeam
        // DO NOT UNCOMMENT IF STATEMENT UNTIL WENTAO AND TUTU HAVE CODE FOR BEFORE TAPE DETECTION USING isRedTeam
//        if (gamepad2.y || !cv.isRobotBeforeTape(isRedTeam)) {
            runMotionControl();
            runArmClawControl();

//            inDep.checkForBrake();
            // TODO: uncomment test each method below one-by-one
            // runAprilTagParallelAlignControl();
            // runAprilTagAlignmentControl();
            // teamPropLocationControl();
            // runPixelAlignmentControl();
//        }
    }
}
