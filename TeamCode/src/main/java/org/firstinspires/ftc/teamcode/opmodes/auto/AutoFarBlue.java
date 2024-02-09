package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementStringInterpreter;

@Autonomous(name = "Auto Far Blue")
@Config
public class AutoFarBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;

    public static String sequences = "";
    public static boolean useString = false;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();
        MovementSequence[] seqs = null;
        if (useString)
            seqs = MovementStringInterpreter.toMovementSequenceArray(sequences);

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder().build();
        if (!useString)
            initCV = new MovementSequenceBuilder()
                .forwardLeft(4, 12) // calibrate
                .build();
        else
            initCV = seqs[0];

        drive.followMovementSequence(initCV);
        int propLocation = robot.getCV().getPropLocation();
//        int propLocation = 0;

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                //approachWhite = new MovementSequenceBuilder().build(),
                //placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();


        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;


        if (propLocation == 0) {
            // left
            placePurple = new MovementSequenceBuilder()
                    .forwardLeft(29.5, 5.75)
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(8)
                    .forwardLeft(6, 26)
                    .turnRight(180)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(47)
                    .raiseArm(20, true)
                    .backwardLeft(39, 45)
                    .flipElbow()
                    .raiseArm(200, true)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)
                    .left(31)
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(19)
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();
//            approachFirstWhite = new MovementSequenceBuilder()
//                    .raiseArm(WHITE_PX_HEIGHT) // raise for white pixel
//                    .closeRightClaw()
//                    .backward(10)
//                    .right(15) // move towards white stack (closest to center)
//                    .turnLeft(180) // 180 to face white stack
//                    .openRightClaw()
//                    //.alignWithWhitePixel() // align w/ white stack obv. lol
//                    .sleepFor(500)
//                    .forward(11) // move towards stack
//                    .restElbow()
//                    .closeRightClaw() // intake from stack
//                    .sleepFor(1000)
//                    .backwardLeft(2, 10) // move towards white stack (closest to center)
//                    .build();


//            approachWhite = new MovementSequenceBuilder()
//                    .forward(88) // move to white pixels
//                    .alignWithWhitePixel() // i wonder what this does
////                    .forward(11) // move a bit more after align with white
//                    .closeRightClaw() // intake 2 white pixels
//                    .build();
//            placeWhite = new MovementSequenceBuilder()
//                    .closeBothClaws()
//                    .backward(99) // move towards backdrop
//                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
//                    .backwardRight(11, 24) // move towards backdrop
//                    .openRightClaw() // open right claw to release 1 pixel
//                    .sleepFor(200) // allow 1st pixel to fall
//                    .closeRightClaw() // close right claw to prevent 2nd pixel release
//                    .right(3) // move to drop 2nd pixel
//                    .openRightClaw() // drop 2nd pixel
//                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
//                    .forwardLeft(20, 24) // align with truss
//                    .build();
        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forwardLeft(28.5, 1.75)
                    .openRightClaw()
                    .sleepFor(150)
                    .backwardLeft(25, 2)
                    .turnRight(90)  // -37 60
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(47)
                    .raiseArm(20, true)
                    .backwardLeft(39, 38)
                    .flipElbow()
                    .raiseArm(200, true)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)
                    .left(24)
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(19)
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();

//            approachFirstWhite = new MovementSequenceBuilder()
//                    .turnRight(90) // turn to face white pixel stacks
//                    .forwardLeft(20, 24) // move towards white pixel stack (closest to center one)
//                    .alignWithWhitePixel() // thing
////                    .forward(6) // move to pixel stack
//                    .closeRightClaw() // intake white pixel
//                    .build();
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
//                    .lowerArm(120 - (WHITE_PX_HEIGHT * 3 / 5)) // arm is 120 before this point, now lower to 2nd white pxl pos
//                    .forwardLeft(31, 18) // align with truss
//                    .build();
        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forwardRight(17.5,6.25)
                    .openRightClaw()
                    .sleepFor(150)
                    .backwardLeft(6, 1)
                    .turnRight(90)
                    .backwardRight(9, 8)  // -37 60
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(47)
                    .raiseArm(20, true)
                    .backwardLeft(39, 31)
                    .flipElbow()
                    .raiseArm(200, true)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)
                    .left(17)
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(19)
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();

//            approachFirstWhite = new MovementSequenceBuilder()
//                    .left(15) // move towards white pixel stack (the one closest to the center)
//                    .alignWithWhitePixel() // chicken nugget
////                    .forward(22) // move towards pixel stack
//                    .closeRightClaw() // intake white pixel
//                    .build();
//            approachWhite = new MovementSequenceBuilder()
//                    .forward(66) // move to white pixels
//                    .alignWithWhitePixel() // i wonder what this does
////                    .forward(11) // move a bit more after align with white
//                    .closeRightClaw() // intake 2 white pixels
//                    .build();
//            placeWhite = new MovementSequenceBuilder()
//                    .backward(99) // move towards backdrop
//                    .raiseArm(120 - WHITE_PX_HEIGHT) // raise arm to place pixels, considering the arm is slightly raised at this point (might be raised at level 1 or level 2 doesn't matter prob)
//                    .backwardRight(11, 12) // move towards backdrop
//                    .openRightClaw() // open right claw to release 1 pixel
//                    .sleepFor(200) // allow 1st pixel to fall
//                    .closeRightClaw() // close right claw to prevent 2nd pixel release
//                    .right(3) // move to drop 2nd pixel
//                    .openRightClaw() // drop 2nd pixel
//                    // CONSIDER MAYBE JUST OPENING (at near-vertical angle?), MAYBE IT WILL DROP BOTH IN THE RIGHT SPOTS
//                    .lowerArm(120 - (WHITE_PX_HEIGHT * 2 / 5)) // arm is about 120 before this point, now lower to 2nd white pxl pos
//                    .forwardLeft(42, 24) // align with truss
//                    .build();
        }

        if (useString) {
            placePurple = seqs[1];
            placeYellow = seqs[2];
            park = seqs[3];
        }


        // perform the actual movements here in sequence
        drive.followMovementSequence(placePurple);
//        drive.followMovementSequence(approachFirstWhite);
        drive.followMovementSequence(placeYellow);
//        for (int i = 0;i<2;i++)
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