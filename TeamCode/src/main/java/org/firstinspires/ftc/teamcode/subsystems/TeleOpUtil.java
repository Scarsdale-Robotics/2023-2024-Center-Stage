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
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public boolean speedIsFast = true;
    private boolean clawToggle = false;
    private boolean speedToggle = false;
    private boolean omniToggle = false;
    public boolean omniMode = false;
    public TeleOpUtil(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedTeam, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        robot = new HardwareRobot(hardwareMap);
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
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
                robot.cameraName,
                drive, telemetry, isRedTeam, opMode
        );
        this.isRedTeam = isRedTeam;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private void runArmRigidControl() {
        if (gamepad1.right_bumper)
            inDep.raiseArm();
        else if (gamepad1.left_bumper)
            inDep.lowerArm();
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
//    private void teamPropLocationControl() {
//        if (gamepad1.dpad_left) {
//            int teamPropLocation = cv.getTeamPropLocation();
//            switch (teamPropLocation) {
//                case 0:
//                    // left location
//                    drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
//                    break;
//                case 2:
//                    // right location
//                    drive.driveRobotCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
//                    break;
//                default:
//                    // center location
//                    drive.driveRobotCentric(0, 1 * SpeedCoefficients.getForwardSpeed(), 0);
//                    break;
//            }
//        };
//    }

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

    /**
     * PRIMARY MOTION CONTROL METHOD
     */
    private void runMotionControl() {
        // TOGGLE MOVE SPEED MODE CONTROL
        if ((gamepad1.dpad_up && !speedToggle)) {
            speedToggle = true;
            if (SpeedCoefficients.getMode() == 0) {
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
                speedIsFast = true;
            } else {
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
                speedIsFast = false;
            }
        }
        if (!gamepad1.dpad_up) {
            speedToggle = false;
        }
        //Toggle Omni Mode

        if (gamepad1.dpad_down && !omniToggle) {
            omniToggle = true;
            omniMode = !omniMode;
        }
        if (!gamepad1.dpad_down) omniToggle = false;

        // Drive Robot
        if (!omniMode) {
            double moveInputX = 0;
            double moveInputY = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.6) {
                moveInputX = Math.signum(gamepad1.left_stick_x) * SpeedCoefficients.getStrafeSpeed();
                moveInputY = 0;
            } else if (Math.abs(gamepad1.left_stick_y) > 0.6) {
                moveInputX = 0;
                moveInputY = Math.signum(gamepad1.left_stick_y) * SpeedCoefficients.getForwardSpeed();
            } else {
                moveInputX = gamepad1.left_stick_x;
                moveInputY = gamepad1.left_stick_y;
            }
            // DRIVE CONTROL
            drive.driveRobotCentric(-moveInputX,moveInputY,-gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed()
            );
        } else {
            drive.driveRobotCentric(-gamepad1.left_stick_x * SpeedCoefficients.getStrafeSpeed(),gamepad1.left_stick_y * SpeedCoefficients.getForwardSpeed(),-gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed()
            );
        }

    }

    /**
     * PRIMARY ARM CLAW CONTROL METHOD
     */
    private void runArmClawControl() {
        // CLAW TOGGLE CONTROL

        if (gamepad1.y && !clawToggle) {
            if (inDep.getIsOpen()) inDep.close();
            else inDep.open();

            clawToggle = true;
        }
        if (!gamepad1.y) clawToggle = false;

        // FLEX ARM MOVEMENT MODE CONTROL
        inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());

        // RIGID ARM MOVEMENT MODE CONTROL
//        runArmRigidControl();

        // RESET ARM CONTROL
        if (gamepad1.a || gamepad2.a) {
            inDep.resetArmEncoder();
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
    }

    public void tick() {
        double DISTANCE_BEFORE_BACKBOARD = 5;  // TEMP
        if (gamepad2.y || cv.getAprilTagDistance(isRedTeam ? new Integer[] {4, 5, 6} : new Integer[] {1, 2, 3}) > DISTANCE_BEFORE_BACKBOARD) {
            runMotionControl();
            runArmClawControl();

            // TODO: uncomment test each method below one-by-one
//            runAprilTagParallelAlignControl();
            // runAprilTagAlignmentControl();
//             teamPropLocationControl();
            // runPixelAlignmentControl();
        }
    }
}
