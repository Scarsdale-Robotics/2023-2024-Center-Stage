package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.TeleOpUtil;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "BLUE Drive TeleOp")
public class DriveTeleOpBlue extends LinearOpMode {

    TeleOpUtil teleOp;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    @Override
    public void runOpMode() {
        teleOp = new TeleOpUtil(hardwareMap, telemetry, false, gamepad1, gamepad2, this);

        waitForStart();
        teleOp.robot.getInDep().setLevel(InDepSubsystem.Level.GROUND); // arm, wrist
        teleOp.robot.getInDep().rest(); // elbow
        teleOp.cv.switchCamera(teleOp.cv.cameraName1);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                doCameraSwitching();
                teleOp.tick();
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
