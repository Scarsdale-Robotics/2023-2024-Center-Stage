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
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

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
    public boolean aprilTagAlignToggle = false;
    public boolean alignAprilTagRunning = false;
    public boolean macrosRunning = true;
    public boolean alignPixelRunning = false;
    private Telemetry telemetry;
    private double lastTurnStart;
    private double moveInputX;
    private double moveInputY;
    public int macroCapacity = 0;
    private boolean towardsBackboard = false;
    private final MovementSequence intoBackboardMode;
    private final MovementSequence intoPickupMode;
    private double vs = 0.0;
    private double vf = 0.0;
    private double vt = 0.0;
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

        if (isRedTeam)
        {
            intoBackboardMode = new MovementSequenceBuilder()
                    .turnLeft(90)
                    .build();
            intoPickupMode = new MovementSequenceBuilder()
                    .turnRight(90)
                    .build();
        } else {
            intoBackboardMode = new MovementSequenceBuilder()
                    .turnRight(90)
                    .build();
            intoPickupMode = new MovementSequenceBuilder()
                    .turnLeft(90)
                    .build();
        }

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

    private void epicMacroControl() {
        if (gamepad1.square) {
            drive.followMovementSequence(towardsBackboard ? intoPickupMode : intoBackboardMode);
            towardsBackboard = !towardsBackboard;
        }
    }

    /**
     * PRIMARY MOTION CONTROL METHOD
     */
    private void runMotionControl() {
        // TOGGLE MOVE SPEED MODE CONTROL
        if (gamepad1.dpad_up || gamepad1.circle)
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        if (gamepad1.dpad_down || gamepad1.x)
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);

        // epic macros
        epicMacroControl();

        // drive robot
        double vsn = -gamepad1.left_stick_x * SpeedCoefficients.getStrafeSpeed();
        double vfn = gamepad1.left_stick_y * SpeedCoefficients.getForwardSpeed();
        double vtn = -gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed();
        vs = vsn == 0 ? 0.5 * vs : vsn;
        vf = vfn == 0 ? 0.5 * vf : vfn;
        vt = vtn == 0 ? 0.6 * vt : vtn;
        drive.driveFieldCentric(vs, vf, vt);
    }

    /**
     * PRIMARY ARM CLAW CONTROL METHOD
     */
    private void runArmClawControl() {
        // CLAW TOGGLE CONTROL
        if (gamepad1.y && !clawToggle) {
            if (inDep.getIsLeftClawOpen()) {
                inDep.close();
                // automagically set fast mode after intake
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            } else {
                inDep.open();
                // automagically set fast mode after release
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            }

            clawToggle = true;
        }
        if (!gamepad1.y) clawToggle = false;

        // FLEX ARM MOVEMENT MODE CONTROL
        inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());

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
        if (!gamepad2.x && cvDist < DISTANCE_BEFORE_BACKBOARD && !inDep.getIsLeftClawOpen()) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        } else if (gamepad2.x) {
            gamepad1.rumble(500); // big bomboclat
            gamepad2.rumble(500);
        }

        runAprilTagParallelAlignControl();
//             teamPropLocationControl();
//         runPixelAlignmentControl();
    }
}
