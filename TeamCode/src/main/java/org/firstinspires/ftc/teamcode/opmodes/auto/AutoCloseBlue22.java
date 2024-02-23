package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Close Blue 2+2")
@Config
public class AutoCloseBlue22 extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;
    public static double _1a = 21;
    public static double _1b = 12.5;
    public static double _2 = 31;
    public static double WHITE_OFFSET_X = 6;
    public static double _3 = 27;
    public static double _99 = 13.5;
    public static double _22a = 10;
    public static double _22b = 8;
    public static double _4 = 5;
    public static double _5 = 190;
    public static double WHITE_TRAVEL = 80.25;
    public static String sequence = "";
    public static boolean useString = false;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();
        robot.getInDep().setLevel(InDepSubsystem.Level.GROUND); // arm, wrist
        robot.getInDep().rest(); // elbow

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                .sleepFor(1000)
                .build();

        int propLocation = 0;

        if (!useString) {
            drive.followMovementSequence(initCV);
            propLocation = robot.getCV().getPropLocation();
        }
        robot.getCV().disablePropProcessor();

        double turnOffset = 16.5;

        MovementSequence main = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            main = new MovementSequenceBuilder()
                    .forwardLeft(28, 7)
                    .raiseArm(10, true)
                    .openRightClaw()  // purple
                    .sleepFor(500)
                    .backwardLeft(7, 19)
                    .raiseArm(40, true)
                    .turnRight(90)
                    .raiseArm(160, true)
                    .backward(4)
                    .openLeftClaw()  // yellow
                    .sleepFor(500)
                    .forwardLeft(7, 31)
                    .lowerArm(150, true)
                    .forward(87, false, true)
                    .lowerArm(40, true)
                    .alignWithAprilTagParRot(9, turnOffset)
                    .alignWithAprilTagPos(9, 12, 12, turnOffset)
                    .forward(12)
                    .lowerArm(20, true)
                    .closeBothClaws()  // white intake
                    .sleepFor(1000)
                    .backward(99, false, true)
                    .raiseArm(20, true)
                    .backwardRight(3, 18, false, true)
                    .raiseArm(170, true)
                    .backward(7, false, true)
                    .openBothClaws()  // white deposit
                    .sleepFor(1000)
                    .forward(5, false, true)
                    .left(17, false, true)
                    .lowerArm(170, true)
                    .backward(9, false, true)
                    .forward(9, false, true)  // lower arm maneuver
                    .lowerArm(20, true)
                    .build();
        } else if (propLocation == 1) {
            main = new MovementSequenceBuilder()
                    //(16.75,63.5,-90)
                    .forwardLeft(25.5, 2.25)
                    .raiseArm(10, true)
                    .openRightClaw()  // purple
                    .sleepFor(500)
                    .turnRight(90)
                    .backwardLeft(24, 7)
                    .raiseArm(205, true)
                    .backward(5)
                    .openLeftClaw()  // yellow
                    .sleepFor(1000)
                    .forward(3)
                    .forwardLeft(15, 24)
                    .lowerArm(146, true)
                    .forward(72.5)
                    .lowerArm(69, true)
                    .forwardRight(15.5, 1.5)
                    .closeBothClaws()  // white intake
                    .sleepFor(500)
                    .openBothClaws()
                    .sleepFor(500)
                    .closeBothClaws()  // white intake 2_0
                    .sleepFor(777)
                    .closeBothClaws()  // don't worry about the suspiciously
                    .backwardRight(3, 0)
                    .raiseArm(10)
                    .backwardRight(1, 0)
                    .raiseArm(15)
                    .closeBothClaws()  // high number of closeBothClaws()
                    .flipElbow()
                    .sleepFor(777)
                    .closeBothClaws()  // keep them
                    .restElbow()
                    .backwardRight(88, 2.5)
                    .backwardRight(16.75, 24)
                    .raiseArm(165, true)
                    .backward(7)
                    .openBothClaws()  // white deposit
                    .sleepFor(1000)
                    .forward(3, false, true)
                    .left(24, false, true)
                    .lowerArm(170, true)
                    .backward(11, false, true)
                    .forward(4, false, true)  // lower arm maneuver
                    .lowerArm(20, true)
                    .build();
        } else if (propLocation == 2) {
            main = new MovementSequenceBuilder()
                    .forward(34)
                    .turnRight(90)
                    .forward(6)
                    .raiseArm(10, true)
                    .openRightClaw()  // purple
                    .sleepFor(500)
                    .backward(32)
                    .raiseArm(200, true)
                    .backward(3)
                    .openLeftClaw()  // yellow
                    .sleepFor(500)
                    .forwardLeft(17.68, 17.68, false, true)
                    .lowerArm(190, true)
                    .forward(58, false, true)
                    .forward(30)
                    .lowerArm(20, true)
                    .closeBothClaws()  // white intake
                    .sleepFor(1000)
                    .backward(88, false, true)
                    .raiseArm(20, true)
                    .backwardRight(14.68, 25.68, false, true)
                    .raiseArm(170, true)
                    .backward(6)
                    .openBothClaws()  // white deposit
                    .sleepFor(1000)
                    .forward(3, false, true)
                    .left(26, false, true)
                    .lowerArm(170, true)
                    .backward(10, false, true)
                    .forward(10, false, true)  // lower arm maneuver
                    .lowerArm(20, true)
                    .build();
        }

        // if string is being used, only run string
        if (useString) {
            MovementSequence seq = new MovementSequenceBuilder()
                    .append(sequence)
                    .build();
            drive.followMovementSequence(seq);
        }

        // follow regular if string is not being used
        if (!useString) {
            // perform the actual movements here in sequence
            drive.followMovementSequence(main);
        }

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