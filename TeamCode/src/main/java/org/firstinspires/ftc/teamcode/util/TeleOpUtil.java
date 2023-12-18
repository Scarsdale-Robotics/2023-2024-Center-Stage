package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

public class TeleOpUtil {
    public DriveSubsystem drive;
    public InDepSubsystem inDep;
    public CVSubsystem cvFront;
    public CVSubsystem cvBack;
    private final boolean isRedTeam;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public boolean speedIsFast = true;
    private boolean clawToggle = false;
    private boolean speedToggle = false;
    private boolean omniToggle = false;
    public boolean omniMode = false;
    public boolean aprilTagAlignToggle = false;
    public boolean alignAprilTagRunning = false;
    public boolean macrosRunning = true;
    public boolean alignPixelRunning = false;
    private Telemetry telemetry;
    private double lastTurnStart;
    private double moveInputX;
    private double moveInputY;
    public int macroCapacity = 0;

    public TeleOpUtil(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedTeam, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        RobotSystem robot = new RobotSystem(hardwareMap, isRedTeam, opMode, telemetry);
        drive = robot.getDrive();
        cvFront = robot.getCVFront();
        cvBack = robot.getCVBack();
        this.isRedTeam = isRedTeam;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.lastTurnStart = robot.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void runArmRigidControl() {
        if (gamepad1.left_bumper) {
            inDep.lowerArm();
        } else if (gamepad1.right_bumper) {
            inDep.raiseArm();
        }
    }

    private void macroFailSafe() {
        macroCapacity++;
        if (gamepad1.dpad_left) {
            macrosRunning = false;
            macroCapacity = 0;
        } else if (macroCapacity > 6 && macrosRunning == false) {
            macrosRunning = true;
        }

    }

    private void runAprilTagParallelAlignControl() {
        // iterative version
        if (((gamepad1.b && aprilTagAlignToggle == false) || alignAprilTagRunning == true) && macrosRunning) {
            aprilTagAlignToggle = true;
            alignAprilTagRunning = true;
            //cv.alignParallelWithAprilTag(isRedTeam ? 5 : 2);
            double rotOff = cvBack.getAprilTagRotationalOffset(isRedTeam ? 5 : 2);
            if (Math.abs(rotOff) < cvBack.ERROR_ALIGNMENT) {
                alignAprilTagRunning = false;
            } else if (rotOff < 0 && rotOff != cvBack.NO_ROTATIONAL_OFFSET) {
                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed());
            } else if (rotOff > 0 && rotOff != cvBack.NO_ROTATIONAL_OFFSET) {
                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed());
            }
        }
        if ((!gamepad1.b && !alignAprilTagRunning) || !macrosRunning) {
            aprilTagAlignToggle = false;
            alignAprilTagRunning = false;
        }
    }

    /**
     * PRIMARY MOTION CONTROL METHOD
     */
    private void runMotionControl() {
        // TOGGLE MOVE SPEED MODE CONTROL
        if (gamepad1.dpad_up)
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        if (gamepad1.dpad_down)
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);

        //Toggle Omni Mode
        if (gamepad1.square && !omniToggle) {
            omniToggle = true;
            omniMode = !omniMode;
        }
        if (!gamepad1.square) omniToggle = false;

        // Drive Robot
        if (!omniMode) {
            moveInputX = 0;
            moveInputY = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.6) {
                moveInputX = Math.signum(gamepad1.left_stick_x) * SpeedCoefficients.getStrafeSpeed();
                moveInputY = 0;
                // TODO: consider: might need a cancel button for below
//                if (moveInputX == -1) {
//                    SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
//                }
            } else if (Math.abs(gamepad1.left_stick_y) > 0.6) {
                moveInputX = 0;
                moveInputY = Math.signum(gamepad1.left_stick_y) * SpeedCoefficients.getForwardSpeed();
            } else {
                moveInputX = gamepad1.left_stick_x;
                moveInputY = gamepad1.left_stick_y;
            }
            double turnInput = gamepad1.right_stick_x;
            // DRIVE CONTROL
            drive.driveRobotCentric(-moveInputX,moveInputY,-turnInput * SpeedCoefficients.getTurnSpeed()
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
            if (inDep.getIsLeftClawOpen())
                inDep.close();
            else {
                inDep.open();
                // automagically set fast mode after release
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            }

            clawToggle = true;
        }
        if (!gamepad1.y) clawToggle = false;

        // FLEX ARM MOVEMENT MODE CONTROL
//        inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());

        // RIGID ARM MOVEMENT MODE CONTROL
        runArmRigidControl();

        // RESET ARM CONTROL
        if (gamepad2.a) {
            inDep.resetArmEncoder();
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
    }

    public void tick() {
        double DISTANCE_BEFORE_BACKBOARD = 45;  // TEMP
        double cvDist = cvBack.getAprilTagDistance(isRedTeam ? new Integer[] {4, 5, 6} : new Integer[] {1, 2, 3});
        telemetry.addData("cvDist:", cvDist);
        telemetry.addData("arm pos:", inDep.getArmPosition());
        telemetry.addData("claw open:", inDep.getIsLeftClawOpen());
        telemetry.addData("пуяза 你好何余安 ???", "θωθ");
        telemetry.update();
        runMotionControl();
        runArmClawControl();
        if (!gamepad2.x && !gamepad1.x && cvDist < DISTANCE_BEFORE_BACKBOARD && !inDep.getIsLeftClawOpen()) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        } else if (gamepad1.x || gamepad2.x) {
            gamepad1.rumble(500); // big bomboclat
            gamepad2.rumble(500);
        }

        runAprilTagParallelAlignControl();
//             teamPropLocationControl();
//         runPixelAlignmentControl();
    }
}
