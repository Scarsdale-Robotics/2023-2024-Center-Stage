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

@Autonomous(name = "Auto Close Red")
@Config
public class AutoCloseRed extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;

    public static String sequence = "";
    public static boolean useString = false;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, true, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();
        robot.getInDep().setLevel(InDepSubsystem.Level.GROUND); // arm, wrist
        robot.getInDep().rest(); // elbow

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                .forward(3) // calibrate
                .left(9.5)
                .build();
        drive.followMovementSequence(initCV);
        int propLocation = robot.getCV().getPropLocation();
        robot.getCV().disablePropProcessor();

//        if (!useString) {
//            drive.followMovementSequence(initCV);
//            propLocation = robot.getCV().getPropLocation();
//        }

        MovementSequence purpleYellow = new MovementSequenceBuilder().build(),
                white = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            // (16.75,-63.5,-90)
            purpleYellow = new MovementSequenceBuilder()
                    .forwardRight(28.5, 15.25)
                    .raiseArm(10, true)  // 10
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(222)
                    .backwardLeft(13, 7)
                    .raiseArm(205, true)  // 215
                    .openLeftClaw()
                    .sleepFor(222)
                    .build();
            park = new MovementSequenceBuilder()
                    .left(18)
                    .lowerArm(195)  // 20
                    .backward(11)
                    .forward(6)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(65, 21)
                    .lowerArm(210, true)  // 5
                    .forwardRight(40, 26)
                    .closeBothClaws()
                    .sleepFor(345)
                    .backwardLeft(3, 2)
                    .raiseArm(10, true)  // 15
                    .closeBothClaws()
                    .flipElbow()
                    .sleepFor(321)
                    .closeBothClaws()
                    .restElbow()
                    .backwardLeft(37, 24)
                    .backwardRight(69, 22)
                    .raiseArm(175, true)  // 190
                    .openBothClaws()
                    .sleepFor(555)
                    .lowerArm(170)  // 20
                    .forward(3)
                    .lowerArm(20, true)  // 0
                    .build();
        } else if (propLocation == 1) {
            purpleYellow = new MovementSequenceBuilder()
                    .forwardRight(37.5, 7.25)
                    .raiseArm(10, true)  // 10
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(222)
                    .backwardLeft(20, 10)
                    .raiseArm(205, true)  // 215
                    .openLeftClaw()
                    .sleepFor(222)
                    .build();
            park = new MovementSequenceBuilder()
                    .left(24)
                    .lowerArm(195, true)  // 20
                    .backward(14)
                    .forward(10)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardRight(104, 1)
                    .lowerArm(210, true)  // 5
                    .closeBothClaws()
                    .sleepFor(321)
                    // TODO- TEST: flip elbow while backwards
                    .flipElbow()
                    .backward(4)
                    .raiseArm(10, true)  // 15
                    .restElbow()
                    .backwardRight(80, 0)
                    .backwardRight(22, 4)
                    .raiseArm(175, true)  // 190
                    .openBothClaws()
                    .sleepFor(555)
                    .forwardLeft(98, 2)
                    .lowerArm(165, true)  // 25
                    .forwardRight(8, 8)
                    .lowerArm(20, true)  // 5
                    .closeBothClaws()
                    .sleepFor(321)
                    .flipElbow()
                    .backward(4)
                    .raiseArm(10, true)  // 15
                    .restElbow()
                    .backwardLeft(5, 9)
                    .backward(70)
                    .backward(27)
                    .raiseArm(175, true)  // 190
                    .openBothClaws()
                    .sleepFor(321)
                    .lowerArm(170)
                    .forward(4)
                    .lowerArm(20, true)
                    .build();
        } else {
            purpleYellow = new MovementSequenceBuilder()
                    .forwardLeft(28.5, 8.75)
                    .raiseArm(10, true)  // 10
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(222)
                    .backwardRight(36, 5)
                    .raiseArm(205, true)  // 215
                    .openLeftClaw()
                    .sleepFor(222)
                    .build();
            park = new MovementSequenceBuilder()
                    .left(30)
                    .lowerArm(195, true)  // 20
                    .backward(14)
                    .forward(10)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(39, 31)
                    .lowerArm(190, true)  // 25
                    .forward(39)
                    .lowerArm(20, true)  // 5
                    .forwardRight(26, 24)
                    .closeBothClaws()
                    .sleepFor(321)
                    // TODO check this new ordering for flip optimization (same as above one btw)
                    .flipElbow()
                    .backwardLeft(3, 3)
                    .raiseArm(10, true)  // 15
                    .restElbow()
                    .backwardLeft(23, 21)
                    .backward(39)
                    .backwardRight(40, 20)
                    .raiseArm(200, true)  // 215
                    .openBothClaws()
                    .sleepFor(321)
                    .lowerArm(195)  // 20
                    .forward(3)
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
            drive.followMovementSequence(purpleYellow);
//            drive.followMovementSequence(white);
            drive.followMovementSequence(park);
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