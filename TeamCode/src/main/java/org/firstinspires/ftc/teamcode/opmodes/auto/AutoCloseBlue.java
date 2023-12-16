package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Close Blue")
public class AutoCloseBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot robot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cv;
    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                robot.arm1,
                robot.arm2,
                robot.rightClaw,
                robot.leftClaw,
                robot.wrist,
                robot.elbow,
                this,
                telemetry
        );
        drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                inDep,
                this
        );
        cv = new CVSubsystem(robot.camera,
                robot.cameraName,
                drive,
                telemetry,
                false,
                this);
        inDep.closeClaws();
        runtime.reset();

        waitForStart();

        sleepFor(500);

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                .left(4.16) // Strafe left
                .forward(10.80) // Move forward
                .build();
        drive.followMovementSequence(initCV);
        int propLocation = cv.getPropLocation();

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                approachWhite = new MovementSequenceBuilder().build(),
                placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();


        if (propLocation == 0) {
            // left
            placeYellow = new MovementSequenceBuilder()
                    .forwardLeft(20, 46) // place yellow first, move to backdrop
                    .turnRight(90) // face back to backdrop to place on backdrop
                    .raiseArm(120) // raise arm to place pixel
                    .openClawLeft() // drop yellow pixel
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .forwardLeft(7.65, 5.67) // move forward to purple pixel loc
                    .lowerArm(120) // lower arm
                    .openClawRight() // drop purple pixel
                    .raiseArm(11) // raise arm to not hit purple pixel and to align with white pixel
                    // lets run a test to see if we can try not lowering the arm
                    .backwardLeft(10,24) // move to align with truss
                    .build();
            approachWhite = new MovementSequenceBuilder()
                    .forward(88) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeClawRight() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 24) // move towards backdrop
                    .raiseArm(120) // raise arm to place pixels
                    .openClawRight() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeClawRight() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openClawRight() // drop 2nd pixel
                    .forwardLeft(20, 24) // align with truss
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(35) // drive to park
                    .build();

        } else if (propLocation == 1) {
            approachTape = new MovementSequenceBuilder()
                    .forward(25.77) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .backward(1.70) // Move backward to not hit pixel on turn
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .raiseArm(57.14) // Raise claw
                    .turnLeft(80) // Turn left
                    .left(14.4) // Strafe left
                    .forward(34.29) // Move forward
                    .turnRight(180) // Turn right
                    .backward(16.98) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();


        } else if (propLocation == 2) {
            approachTape = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnRight(87.0) // Turn right
                    .backward(0.31) // Brake
                    .left(3.2) // Strafe left
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .raiseArm(57.14) // Raise claw
                    .backward(6.85) // Move backward
                    .right(20.4) // Strafe right
                    .backward(43.21) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placeYellow);
        drive.followMovementSequence(placePurple);
        for (int i=0;i<2;i++)
        {
            drive.followMovementSequence(approachWhite);
            drive.followMovementSequence(placeWhite);
        }
        drive.followMovementSequence(park);

        stopRobot();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }

    public void stopRobot() {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}