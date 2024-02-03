package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Close Blue")
public class AutoCloseBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;
    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                .left(4.16) // Strafe left
                .forward(10.80) // Move forward
                .build();
        drive.followMovementSequence(initCV);
//        int propLocation = robot.getCVFront().getPropLocation();
        int propLocation = 0;

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
                    .raiseArm(60)
                    .turnLeft(90, true)
                    .left(20)
                    .openLeftClaw()
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forwardLeft(20, 8)
                    .openRightClaw() // drop purple pixel
                    .build();
//            approachWhite = new MovementSequenceBuilder()
//                    .forward(88) // move to white pixels
//                    .alignWithWhitePixel() // i wonder what this does
////                    .forward(11) // move a bit more after align with white
//                    .closeRightClaw() // intake 2 white pixels
//                    .build();
//            placeWhite = new MovementSequenceBuilder()
//                    .backward(99) // move towards backdrop
//                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
//                    .backwardRight(11, 24) // move towards backdrop
//                    .openRightClaw() // open right claw to release 1 pixel
//                    .sleepFor(200) // allow 1st pixel to fall
//                    .closeRightClaw() // close right claw to prevent 2nd pixel release
//                    .right(3) // move to drop 2nd pixel
//                    .openRightClaw() // drop 2nd pixel
//                    .forwardLeft(20, 24) // align with truss
//                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
//                    .build();
            park = new MovementSequenceBuilder()
                    .backwardLeft(5, 24)
                    .lowerArm(60, true)
                    .turnRight(90)
                    .build();

        } else if (propLocation == 1) {

            placeYellow = new MovementSequenceBuilder()
                    .raiseArm(60)
                    .turnLeft(90, true)
                    .forwardLeft(11, 28)
                    .openLeftClaw()
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forward(30)
                    .openRightClaw()
                    .build();
//            approachWhite = new MovementSequenceBuilder()
//                    .forward(77) // move to white pixels
//                    .alignWithWhitePixel() // i wonder what this does
////                    .forward(11) // move a bit more after align with white
//                    .closeRightClaw() // intake 2 white pixels
//                    .build();
//            placeWhite = new MovementSequenceBuilder()
//                    .backward(99) // move towards backdrop
//                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
//                    .backwardRight(11, 24) // move towards backdrop
//                    .openRightClaw() // open right claw to release 1 pixel
//                    .sleepFor(200) // allow 1st pixel to fall
//                    .closeRightClaw() // close right claw to prevent 2nd pixel release
//                    .right(3) // move to drop 2nd pixel
//                    .openRightClaw() // drop 2nd pixel
//                    .forwardLeft(31, 18) // align with truss
//                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
//                    .build();
            park = new MovementSequenceBuilder()
                    .backwardLeft(5, 28)
                    .lowerArm(60, true)
                    .turnRight(90)
                    .build();

        } else if (propLocation == 2) {
            placeYellow = new MovementSequenceBuilder()
                    .backward(48)
                    .turnRight(70, true)
                    .raiseArm(222, true)
                    .openLeftClaw()
                    .build();
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forward(24)
                    .turnRight(20, true)
                    .openRightClaw()
                    .build();
            approachWhite = new MovementSequenceBuilder()
                    .forward(66) // move to white pixels
                    .alignWithWhitePixel() // i wonder what this does
//                    .forward(11) // move a bit more after align with white
                    .closeRightClaw() // intake 2 white pixels
                    .build();
            placeWhite = new MovementSequenceBuilder()
                    .backward(99) // move towards backdrop
                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
                    .backwardRight(11, 12) // move towards backdrop
                    .openRightClaw() // open right claw to release 1 pixel
                    .sleepFor(200) // allow 1st pixel to fall
                    .closeRightClaw() // close right claw to prevent 2nd pixel release
                    .right(3) // move to drop 2nd pixel
                    .openRightClaw() // drop 2nd pixel
                    .forwardLeft(42, 24) // align with truss
                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
                    .build();
            park = new MovementSequenceBuilder()
                    .backwardLeft(5, 32)
                    .lowerArm(60, true)
                    .turnRight(90)
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placePurple);
        drive.followMovementSequence(placeYellow);
//        for (int i=0;i<WHITE_REPS;i++)
//        {
//            drive.followMovementSequence(approachWhite);
//            drive.followMovementSequence(placeWhite);
//        }
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