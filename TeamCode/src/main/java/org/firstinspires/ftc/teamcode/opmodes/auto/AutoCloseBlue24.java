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

@Autonomous(name = "Auto Close Blue 2+4")
@Config
public class AutoCloseBlue24 extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;
    public static double WHITE_TRAVEL = 81;
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

        MovementSequence main = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            main = new MovementSequenceBuilder()
                    .forwardLeft(21, 11)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(10)
                    .left(7)
                    .turnRight(90)
                    .left(10)
                    .raiseArm(210, true)
                    .flipElbow()
                    .sleepFor(250)
                    .backward(12)
                    .openLeftClaw()
                    .sleepFor(333)
                    .forward(5)
                    .restElbow()
                    .left(31)
                    .lowerArm(200, true)
                    .forwardLeft(WHITE_TRAVEL, 5)
                    .forward(10)
                    .lowerArm(10, true)
                    .closeBothClaws()
                    .sleepFor(500)
                    .backward(10)
                    .raiseArm(10, true)
                    .backwardRight(WHITE_TRAVEL, 5)
                    .right(22)
                    .raiseArm(190, true)
                    .closeBothClaws()
                    .flipElbow()
                    .sleepFor(250)
                    .backward(8)
                    .openBothClaws()
                    .sleepFor(1000)
//                    .forward(5)
//                    .left(19)
//                    .restElbow()
//                    .lowerArm(210, true)
//                    .forward(WHITE_TRAVEL)
//                    .forward(5)
////                    .lowerArm(10, true)
//                    .closeBothClaws()
//                    .sleepFor(500)
//                    .backward(10)
//                    .raiseArm(10, true)
//                    .backward(WHITE_TRAVEL-10+5)
//                    .right(19)
//                    .raiseArm(200, true)
//                    .flipElbow()
//                    .sleepFor(250)
//                    .backward(7)
//                    .openBothClaws()
//                    .sleepFor(150)
                    .forward(5)
                    .restElbow()
                    .left(20)
                    .lowerArm(190, true)
                    .backward(15)
                    .forward(5)
                    .lowerArm(10, true)
                    .build();
        } else if (propLocation == 1) {
            main = new MovementSequenceBuilder()
                    .forward(30)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(8)
                    .turnRight(90)
                    .backwardLeft(22, 6)
                    .backwardLeft(5, 1)
                    .forward(6)
                    .left(23)
                    .forwardLeft(97, 1)
                    .backwardRight(97, 1)
                    .right(17)
                    .backward(6)
                    .forward(6)
                    .left(17)
                    .forwardLeft(97, 1)
                    .backwardRight(97, 1)
                    .right(17)
                    .backward(6)
                    .forward(6)
                    .left(19)
                    .backward(11)
                    .build();
        } else if (propLocation == 2) {
            main = new MovementSequenceBuilder()
                    .forwardRight(28.5, 6.75)
                    .turnRight(90)
                    .openRightClaw()
                    .sleepFor(150)
                    .backwardRight(13, 1)
                    .turnLeft(180)
                    .raiseArm(10, true)
                    .forwardLeft(13, 1)
                    .closeRightClaw()
                    .sleepFor(500)
                    .backwardLeft(28, 23)
                    .backward(47)
                    .raiseArm(10, true)
                    .backwardRight(29, 17)
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(4)
                    .openLeftClaw()
                    .sleepFor(150)
                    .openRightClaw()
                    .sleepFor(150)
                    .forward(4)
                    .forwardLeft(27, 16)
                    .restElbow()
                    .closeLeftClaw()
                    .lowerArm(210, true)
                    .forward(49)
                    .forwardRight(28, 22)
                    .lowerArm(10, true)
                    .closeRightClaw()
                    .sleepFor(500)
                    .backwardLeft(29, 22)
                    .raiseArm(10, true)
                    .backward(47)
                    .raiseArm(10, true)
                    .backwardRight(28, 29)
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(4)
                    .openRightClaw()
                    .sleepFor(150)
                    .forward(4)
                    .right(16)
                    .restElbow()
                    .closeRightClaw()
                    .lowerArm(220, true)
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