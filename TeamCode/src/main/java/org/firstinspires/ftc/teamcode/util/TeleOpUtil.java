package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

public class TeleOpUtil {
    public DriveSubsystem drive;
    public InDepSubsystem inDep;
    //    public CVSubsystem cvFront;
    public CVSubsystem cv;
    public EndgameSubsystem endgame;
    private final boolean isRedTeam;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public boolean speedIsFast = true;
    private boolean clawLeftToggle = false;
    private boolean clawRightToggle = false;
    private boolean speedToggle = false;
    public boolean aprilTagAlignToggle = false;
    public boolean alignAprilTagRunning = false;
    public boolean macrosRunning = true;
    public boolean alignPixelRunning = false;
    private Telemetry telemetry;
    private double lastTurnStart;
    private double moveInputX;
    private double moveInputY;
    boolean elbowToggle = false, droneToggle = false, elbowClosed = false, resetArmEncoderToggle = false;
    private double vs = 0.0;
    private double vf = 0.0;
    private double vt = 0.0;

    public int macroCapacity = 0;
    private boolean towardsBackboard = false;
    private final MovementSequence intoBackboardMode;
    private final MovementSequence intoPickupMode;
    public RobotSystem robot;
    public TeleOpUtil(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedTeam, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        robot = new RobotSystem(hardwareMap, isRedTeam, opMode, telemetry);
        drive = robot.getDrive();
        cv = robot.getCV();
        cv.disablePropProcessor();
        cv.decShutter();
        cv.switchCamera(cv.cameraName1);
        this.isRedTeam = isRedTeam;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.lastTurnStart = robot.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        this.inDep = robot.getInDep();
        this.endgame = robot.getEndgame();
        inDep.open();
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
//        if (gamepad1.left_bumper) {
//            inDep.lowerArm();
//        } else if (gamepad1.right_bumper) {
//            inDep.raiseArm();
//        }
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

//    private void runAprilTagParallelAlignControl() {
//        // iterative version
//        if (((gamepad1.b && aprilTagAlignToggle == false) || alignAprilTagRunning == true) && macrosRunning) {
//            aprilTagAlignToggle = true;
//            alignAprilTagRunning = true;
//            //cv.alignParallelWithAprilTag(isRedTeam ? 5 : 2);
//            double rotOff = cvBack.getAprilTagRotationalOffset(isRedTeam ? 5 : 2);
//            if (Math.abs(rotOff) < cvBack.ERROR_ALIGNMENT) {
//                alignAprilTagRunning = false;
//            } else if (rotOff < 0 && rotOff != cvBack.NO_ROTATIONAL_OFFSET) {
//                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed());
//            } else if (rotOff > 0 && rotOff != cvBack.NO_ROTATIONAL_OFFSET) {
//                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed());
//            }
//        }
//        if ((!gamepad1.b && !alignAprilTagRunning) || !macrosRunning) {
//            aprilTagAlignToggle = false;
//            alignAprilTagRunning = false;
//        }
//    }

    private void epicMacroControl() {
        if (gamepad1.square) {
            drive.followMovementSequence(towardsBackboard ? intoPickupMode : intoBackboardMode);
            towardsBackboard = !towardsBackboard;
        }
    }

    private void automaticIntakeControl() {
        if (gamepad1.circle) {
            // TODO: IMPLEMENT ONCE CAMERAS READY
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

        // epic macros
//        epicMacroControl();

        // drive robot
        double vsn = gamepad1.left_stick_x * SpeedCoefficients.getStrafeSpeed();
        double vfn = -gamepad1.left_stick_y * SpeedCoefficients.getForwardSpeed();
        double vtn = gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed();
//        double MOMENTUM_FACTOR = 0.1;  // higher = less momentum
//        if (vsn == 0) vs = Math.abs(vs) < 0.001 ? 0 : (vsn * MOMENTUM_FACTOR + vs * (1-MOMENTUM_FACTOR)); else vs = vsn;
//        if (vfn == 0) vf = Math.abs(vf) < 0.001 ? 0 : (vfn * MOMENTUM_FACTOR + vf * (1-MOMENTUM_FACTOR)); else vf = vfn;
//        if (vtn == 0) vt = Math.abs(vt) < 0.001 ? 0 : (vtn * MOMENTUM_FACTOR + vt * (1-MOMENTUM_FACTOR)); else vt = vtn;  // could add a constant here to adjust for unintended turns
//        drive.driveFieldCentric(vs, vf, vt);

//        if (Math.abs(vsn) > 0.6) {
//            vsn = Math.signum(vsn);
//            vfn=0;
//        }
//        if (Math.abs(vfn) > 0.6) {
//            vfn = Math.signum(vfn);
//            vsn=0;
//        }

        // TODO: test if there is delay only when switching forward/backward (if so prolly below code)
//        if (vfn < 0 && vf >= 0) {
//            new Thread(() -> cv.switchCamera(cv.cameraName1));
////            cv.switchCamera(cv.cameraName1);
//        } else if (vfn > 0 && vf <= 0) {
//            new Thread(() -> cv.switchCamera(cv.cameraName2));
////            cv.switchCamera(cv.cameraName2);
//        }

        vs=vsn;
        vf=vfn;
        vt=vtn;

        //TODO: GET EXTERNAL IMU FOR FIELD CENTRIC
//        drive.driveFieldCentric(vsn, vfn, vtn);
        drive.driveRobotCentric(vs, vf, vt);
    }
//HHHHHHHHHHIIIIIIIII
    //poopypoop poop
    /**/

    /**
     * PRIMARY ARM CLAW CONTROL METHOD
     */
    private void runArmClawControl() {
        // CLAW TOGGLE CONTROL
        if (gamepad1.y && !clawLeftToggle && !clawRightToggle) {
            if (!inDep.getIsLeftClawOpen() || !inDep.getIsRightClawOpen()) {
                inDep.open();
            } else {
                inDep.close();
            }
            clawLeftToggle = true;
            clawRightToggle = true;
        }
        if (gamepad1.square && !clawLeftToggle) {
            if (inDep.getIsLeftClawOpen()) {
                inDep.closeLeft();
                // automagically set fast mode after intake
//                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            } else {
                inDep.openLeft();
                // automagically set fast mode after release
//                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            }
            clawLeftToggle = true;
        }
        if (gamepad1.circle && !clawRightToggle) {
            if (inDep.getIsRightClawOpen()) {
                inDep.closeRight();
                // automagically set fast mode after intake
                //                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            } else {
                inDep.openRight();
                // automagically set fast mode after release
                //                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            }
            clawRightToggle = true;
        }
        if (!gamepad1.square && !gamepad1.y) clawLeftToggle = false;
        if (!gamepad1.circle && !gamepad1.y) clawRightToggle = false;

        // FLEX ARM MOVEMENT MODE CONTROL
        inDep.rawPower((gamepad1.right_trigger - gamepad1.left_trigger) * SpeedCoefficients.getArmSpeed());

        // RIGID ARM MOVEMENT MODE CONTROL
//        runArmRigidControl();

        // RESET ARM CONTROL
        if ((gamepad2.circle && gamepad2.a) && !resetArmEncoderToggle) {
            inDep.resetArmEncoder();

            gamepad1.rumble(500);
            gamepad2.rumble(500);
            resetArmEncoderToggle = true;
        }
        if (!gamepad2.circle && !gamepad2.a) resetArmEncoderToggle = false;
    }


    /**
     * PRIMARY DRONE CONTROL METHOD
     */
    private void runEndgameControl() {
        if ((gamepad2.triangle && gamepad2.square) && !droneToggle) {
            endgame.releaseDrone();
            droneToggle = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
        if (!gamepad2.triangle && !gamepad2.square) {
            endgame.holdDrone();
            droneToggle = false;
        }
    }

    public void tick() {
        double DISTANCE_BEFORE_BACKBOARD = 45;  // TEMP
        double cvDist = cv.getAprilTagDistance(isRedTeam ? new Integer[] {4, 5, 6} : new Integer[] {1, 2, 3});
        telemetry.addData("cvDist:", cvDist);
        telemetry.addData("arm pos:", inDep.getLeftArmPosition());
        telemetry.addData("delta:", inDep.getDelta());
        telemetry.addData("claw open:", inDep.getIsLeftClawOpen());
        telemetry.addData("пуяза 你好何余安 ???", "θωθ");
        telemetry.addData("LOC X", cv.getPosition(0, 0)[0]);
        telemetry.addData("LOC Y", cv.getPosition(0, 0)[1]);
        telemetry.addData("rot", cv.getPosition(0, 0)[2]);
        telemetry.addData("YAW", drive.getYaw());
        telemetry.addData("IMU name",drive.getIMU().getDeviceName());
        telemetry.update();
        runMotionControl();
        runArmClawControl();
        runEndgameControl();

//        //TODO: does this cause switch between "robot" centric and field centric
//        if (gamepad2.circle && gamepad2.dpad_left) {
////            drive.resetIMU();
//            telemetry.addData("poop", Math.random());
//        }

        if (!gamepad2.dpad_left && cvDist < DISTANCE_BEFORE_BACKBOARD && !(inDep.getIsLeftClawOpen() || inDep.getIsRightClawOpen())) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        } else {
            if (gamepad2.dpad_up) SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            else SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        }

        if (gamepad2.dpad_left) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }


//        runAprilTagParallelAlignControl();
//             teamPropLocationControl();
//         runPixelAlignmentControl();
    }
}