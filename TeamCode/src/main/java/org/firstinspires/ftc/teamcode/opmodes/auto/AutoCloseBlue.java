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
    public void runOpMode() {
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
        inDep.close();
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

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            // left
            placeYellow = new MovementSequenceBuilder()
                    .forwardLeft(20, 46) // place yellow first, move to backdrop
                    .turnRight(90) // face back to backdrop to place on backdrop
                    .raiseArm(120) // raise arm to place pixel
                    .openClawLeft() // drop yellow pixel
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .lowerArm(120) // lower arm
                    .forwardLeft(7.65, 15.67) // move forward to purple pixel loc
                    .openClawRight() // drop purple pixel
                    .raiseArm(WHITE_PX_HEIGHT) // raise arm to not hit purple pixel and to align with white pixel
                    // lets run a test to see if we can try not lowering the arm
                    .backwardLeft(5, 15) // move to align with truss
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
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openClawRight() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeClawRight() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openClawRight() // drop 2nd pixel
                    .forwardLeft(20, 24) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(35) // drive to park
                    .build();

        } else if (propLocation == 1) {

            placeYellow = new MovementSequenceBuilder()
                    .forwardLeft(26, 52) // place yellow first, move to backdrop
                    .turnRight(90) // face back to backdrop to place on backdrop
                    .raiseArm(120) // raise arm to place pixel
                    .openClawLeft() // drop yellow pixel
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .lowerArm(120) // lower arm
                    .forwardLeft(15.6789012345, 7.5643210987) // move forward to purple pixel loc
                    .openClawRight() // drop purple pixel
                    .raiseArm(WHITE_PX_HEIGHT) // raise arm to not hit purple pixel and to align with white pixel
                    // lets run a test to see if we can try not lowering the arm now
                    .left(15) // move to align with truss
                    .build();
            approachWhite = new MovementSequenceBuilder()
                    .forward(77) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeClawRight() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 24) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openClawRight() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeClawRight() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openClawRight() // drop 2nd pixel
                    .forwardLeft(31, 18) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(46) // drive to park
                    .build();

        } else if (propLocation == 2) {
            placeYellow = new MovementSequenceBuilder()
                    .forwardLeft(32, 52) // place yellow first, move to backdrop
                    .turnRight(90) // face back to backdrop to place on backdrop
                    .raiseArm(120) // raise arm to place pixel
                    .openClawLeft() // drop yellow pixel
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .lowerArm(120) // lower arm
                    .forwardRight(28.8888888888, 2) // move forward to purple pixel loc
                    .openClawRight() // drop purple pixel
                    .raiseArm(WHITE_PX_HEIGHT) // raise arm to not hit purple pixel and to align with white pixel
                    // lets run a test to see if we can try not lowering the arm now
                    .left(15) // move to align with truss
                    .build();
            approachWhite = new MovementSequenceBuilder()
                    .forward(66) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeClawRight() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 12) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openClawRight() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeClawRight() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openClawRight() // drop 2nd pixel
                    .forwardLeft(42, 24) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(57) // drive to park
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placeYellow);
        drive.followMovementSequence(placePurple);
        for (int i=0;i<WHITE_REPS;i++)
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