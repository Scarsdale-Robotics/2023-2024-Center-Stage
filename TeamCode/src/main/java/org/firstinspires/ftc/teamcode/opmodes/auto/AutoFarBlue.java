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

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {
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
        int propLocation = cv.getPropLocation();

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                approachFirstWhite = new MovementSequenceBuilder().build(),
                approachWhite = new MovementSequenceBuilder().build(),
                placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            // left
            placePurple = new MovementSequenceBuilder()
                    .forwardLeft(28, 2) // move towards left spike mark
                    .turnLeft(90) // turn to left spike mark
                    .openRightClaw() // release purple pixel
                    .build();
            approachFirstWhite = new MovementSequenceBuilder()
                    .raiseArm(WHITE_PX_HEIGHT) // raise for white pixel
                    .backwardRight(2, 24) // move towards white stack (closest to center)
                    .turnLeft(180) // 180 to face white stack
                    .alignWhitePixel() // align w/ white stack obv. lol
                    .forward(6) // move towards stack
                    .closeRightClaw() // intake from stack
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(96) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to pixel placement pos
                    .backwardRight(11, 24) // move towards backdrop
                    .openLeftClaw() // release yellow
                    .openRightClaw() // release white
                    .forwardLeft(20, 33) // align with truss to head towards white stack
                    .build(); // build lol
            approachWhite = new MovementSequenceBuilder()
                    .forward(88) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeRightClaw() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 24) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openRightClaw() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeRightClaw() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openRightClaw() // drop 2nd pixel
                    .forwardLeft(20, 24) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(35) // drive to park
                    .build();
        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder()
                    .forward(26.69) // Moving forward toward the pixel placing area
                    .openRightClaw() // place purple
                    .backward(5) // Move backward to not hit pixel on turn
                    .raiseArm(WHITE_PX_HEIGHT) // raise arm to not hit pixel on term
                    .build();
            approachFirstWhite = new MovementSequenceBuilder()
                    .turnRight(90) // turn to face white pixel stacks
                    .forwardLeft(20, 24) // move towards white pixel stack (closest to center one)
                    .alignWhitePixel() // thing
                    .forward(6) // move to pixel stack
                    .closeRightClaw() // intake white pixel
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(96) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to pixel placement pos
                    .backwardRight(11, 18) // move towards backdrop
                    .openLeftClaw() // release yellow
                    .openRightClaw() // release white
                    .forwardLeft(20, 39) // align with truss to head towards white stack
                    .build(); // build lol
            approachWhite = new MovementSequenceBuilder()
                    .forward(77) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeRightClaw() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 24) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openRightClaw() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeRightClaw() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openRightClaw() // drop 2nd pixel
                    .forwardLeft(31, 18) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(46) // drive to park
                    .build();
        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()
                    .forwardLeft(30, 2) // move to right spike mark
                    .turnRight(90) // turn towards right spike mark
                    .openRightClaw() // release purple
                    .raiseArm(WHITE_PX_HEIGHT) // raise arm to not hit pixel on turn
                    .backward(5) // move backward to not collide with pixel
                    .build();
            approachFirstWhite = new MovementSequenceBuilder()
                    .left(15) // move towards white pixel stack (the one closest to the center)
                    .alignWhitePixel() // chicken nugget
                    .forward(22) // move towards pixel stack
                    .closeRightClaw() // intake white pixel
                    .build();
            approachWhite = new MovementSequenceBuilder()
                    .forward(66) // move to white pixels
                    .alignWhitePixel() // i wonder what this does
                    .forward(11) // move a bit more after align with white
                    .closeRightClaw() // intake 2 white pixels
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(96) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to pixel placement pos
                    .backwardRight(11, 12) // move towards backdrop
                    .openLeftClaw() // release yellow
                    .openRightClaw() // release white
                    .forwardLeft(20, 45) // align with truss to head towards white stack
                    .build(); // build lol
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .backwardRight(11, 12) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .openRightClaw() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeRightClaw() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openRightClaw() // drop 2nd pixel
                    .forwardLeft(42, 24) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .turnLeft(90) // turn to post-auto (pre-teleop) ideal pos
                    .left(57) // drive to park
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placePurple);
        drive.followMovementSequence(approachFirstWhite);
        drive.followMovementSequence(placeYellow);
        for (int i = 0;i<2;i++)
        {
            drive.followMovementSequence(approachWhite);
            drive.followMovementSequence(placeWhite);
        }
        drive.followMovementSequence(park);

        drive.stopController();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }
}