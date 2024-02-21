package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.TeleOpUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@TeleOp(name = "BLUE Drive TeleOp")
@Config
public class DriveTeleOpBlue extends LinearOpMode {
    CVSubsystem cv;
    TeleOpUtil teleOp;
    SpeedCoefficients speedCoefficients;
    DriveSubsystem drive;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;

    // length of arm (cm) (from robot to claw)
    public double armLength = 34.0;

    // length of claw (cm) (from touching arm to the end)
    public double clawLength = 16.0;

    // degree slant of the arm from being completely vertical in the ground state.
    public double degreeOfRobotArmInGroundState = 63.0;

    // number ticks per arm revolutions (360 degrees)
    public double armTickPerRevolution = 4062.8;

    // robot height (in cm)
    public double robotHeight = 28.5;

    // current arm position (degrees with 0 degrees being ground)
    public double armPos = 0;

    // current robot distance from backdrop
    public double currentDistance = 0;

    // current target robot distance from backdrop
    public double targetDistance = 0;

    // auto-adjust mode on or off
    public boolean autoAdjustMode = true;

    public static double wantedDistanceFromEndOfArmToBackdrop = 25;

    public double relativeError = 0;

    public static double distanceFromBackdropToStartAutoAdjust = 25;

    public static double errorThreshold = 0.5;

    public static int pnFac = -1;


    @Override
    public void runOpMode() {
        teleOp = new TeleOpUtil(hardwareMap, telemetry, false, gamepad1, gamepad2, this);

        waitForStart();
        teleOp.robot.getInDep().setLevel(InDepSubsystem.Level.GROUND); // arm, wrist
        teleOp.robot.getInDep().rest(); // elbow
        teleOp.cv.switchCamera(teleOp.cv.cameraName1);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                teleOp.tick();
                if (autoAdjustMode) {
                    // find current distance from backdrop
                    currentDistance = teleOp.cv.getAprilTagDistance(1, 2, 3);
                    if (currentDistance < distanceFromBackdropToStartAutoAdjust) {
                        // calculate degree of arm
                        armPos = teleOp.robot.getInDep().getLeftArmPosition() * 360 / armTickPerRevolution + degreeOfRobotArmInGroundState;
                        // calculate target distance from backdrop (can check math in shared doc)
                        // TODO: INVERT COS
                        targetDistance = (wantedDistanceFromEndOfArmToBackdrop * 2 + java.lang.Math.sqrt(3.0) * armLength * java.lang.Math.cos(java.lang.Math.toRadians(270 - armPos)) - robotHeight - armLength * java.lang.Math.sin(java.lang.Math.toRadians(270 - armPos))) / java.lang.Math.sqrt(3.0);
                        // calculate relative error (direction preserved)
                        relativeError = (currentDistance - targetDistance) / targetDistance;
                        if (Math.abs(relativeError) > errorThreshold) {
                            // change speed according to error, so robot moves faster the farther away it is from target (might want to change this to complete PID later)
                            teleOp.drive.driveRobotCentric(0, pnFac * SpeedCoefficients.getForwardSpeed() * relativeError, 0);
                        }
                    }
                }

                //            telemetry.addData("Arm pos: ", teleOp.robot.arm.motor.getCurrentPosition());
                //            telemetry.addData("Wrist pos: ", teleOp.robot.wrist.getPosition());
                //            telemetry.addData("Claw pos: ", teleOp.robot.claw.getPosition());
                //            telemetry.addData("arm power: ", teleOp.robot.arm.motor.getPower());
                //            telemetry.addData("Speed is Fast: ", teleOp.speedIsFast);
                //            telemetry.addData("Omni Mode:", teleOp.omniMode);
                //            telemetry.update();
            }
        }
        teleOp.cv.close();
    }

    /**
     * Set the active camera according to input from the gamepad.
     */
    private void doCameraSwitching() {
        if (teleOp.cv.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // If the left bumper is pressed, use Webcam Back.
            // If the right bumper is pressed, use Webcam Front.
            boolean newLeftBumper = gamepad1.left_bumper;
            boolean newRightBumper = gamepad1.right_bumper;
            if (newLeftBumper && !oldLeftBumper) {
                teleOp.cv.visionPortal.setActiveCamera(teleOp.cv.cameraName1);
            } else if (newRightBumper && !oldRightBumper) {
                teleOp.cv.visionPortal.setActiveCamera(teleOp.cv.cameraName2);
            }
            oldLeftBumper = newLeftBumper;
            oldRightBumper = newRightBumper;
        }

    }   // end method doCameraSwitching()
}
