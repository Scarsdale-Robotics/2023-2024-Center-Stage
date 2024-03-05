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

@Autonomous(name = "[2+2] Auto Close Red")
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
        robot.getInDep().autoInit();
        robot.getCV().autoExposure();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();
//        robot.getInDep().setLevel(InDepSubsystem.Level.GROUND); // arm, wrist
//        robot.getInDep().rest(); // elbow

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
//                .forward(3) // calibrate
//                .left(9.5)
                .build();
//        drive.followMovementSequence(initCV);
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

        if (propLocation == 2) {
            // (16.75,-63.5,-90)
            purpleYellow = new MovementSequenceBuilder()
                    .forwardRight(28.5-4, 15.25+5.5)
                    .raiseArm(3.5, true)  // 3.5
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(123)
                    .backwardLeft(11.5-5, 9)
                    .raiseArm(214.5, true)  // 218
                    .openLeftClaw()
                    .sleepFor(123)
                    .forward(4)
                    .build();
            park = new MovementSequenceBuilder()
                    .left(18)
                    .lowerArm(198)  // 20
                    .backward(11)
                    .forward(6)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(25, 15)
                    .lowerArm(215, true)  // 5
                    .forward(38)
                    .forwardRight(20, 27.5)
                    .forward(5)
                    .closeBothClaws()
                    .sleepFor(123)
                    .backwardLeft(3, 0, false, true)
                    .backwardLeft(3, 0)
                    .raiseArm(10, true)  // 15
                    .closeBothClaws()
                    .flipElbow()
                    .sleepFor(333)
                    .closeBothClaws()
                    .restElbow()
                    .backwardLeft(11, 23, false, true)
                    .backward(60, false, true)
                    .backwardRight(23.5, 26)
                    .raiseArm(175, true)  // 190
                    .openBothClaws()
                    .sleepFor(123)
                    .lowerArm(170)  // 20
                    .forward(3)
                    .lowerArm(20, true)  // 0
                    .build();
        } else if (propLocation == 1) {
            purpleYellow = new MovementSequenceBuilder()
                    .forwardRight(36-2-0.5, 7.25+5.5)
                    .raiseArm(5, true)  // 5
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(111)
                    .backwardLeft(13.5, 9+2.5)
                    .raiseArm(215, true)  // 220
                    .openLeftClaw()
                    .sleepFor(111)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(4)
                    .left(24)
                    .lowerArm(200, true)  // 20
                    .backward(25)
                    .forward(10)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardRight(40+60.5, 4)
                    .lowerArm(218, true)  // 1.5
                    .closeBothClaws()
                    .sleepFor(123)
                    .backward(2, false, true)
                    .backward(4, false, true)
                    .raiseArm(13.5, true)  // 15
                    .flipElbow()
                    .sleepFor(222)
                    .restElbow()
                    .backwardLeft(50, 2.5, false, true)
                    .raiseArm(20, true)  // 35
                    .backwardLeft(40, 9)
                    .raiseArm(170, true)  // 205
                    .openBothClaws()
                    .sleepFor(50)
                    .forwardRight(91-14+3-4-7, 9, false, true)
                    .lowerArm(180, true)  // 25
                    .forwardRight(3+11, 14, false, true)
                    .lowerArm(23, true)  // 2
                    .forwardRight(13, 0)
                    .closeBothClaws()
                    .sleepFor(123)
                    .backwardLeft(1, 1, false, true)
                    .backwardLeft(1, 7, false, true)
                    .raiseArm(13, true)  // 15
                    .flipElbow()
                    .sleepFor(333)
                    .restElbow()
                    .backwardLeft(69+4+6, 5, false, true)
                    .raiseArm(20, true)  // 35
                    .backwardLeft(14.5, 4)
                    .raiseArm(175, true)  // 205
                    .openBothClaws()
                    .sleepFor(50)
                    .lowerArm(180)  // 25
                    .backward(4)
                    .forward(4)
                    .lowerArm(25, true)  // 0
                    .build();
        } else if (propLocation == 0) {
            purpleYellow = new MovementSequenceBuilder()
                    .forwardLeft(27, 8.75-5)
                    .raiseArm(3.5, true)  // 3.5
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(123)
                    .backwardRight(30, 8.5)
                    .raiseArm(211.5, true)  // 215
                    .openLeftClaw()
                    .sleepFor(123)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(4)
                    .left(30)
                    .lowerArm(195, true)  // 20
                    .backward(20)
                    .forward(10)
                    .lowerArm(20, true)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(27, 33)
                    .lowerArm(190, true)  // 25
                    .forward(52)
                    .lowerArm(22, true)  // 3
                    .forwardRight(10, 20)
                    .forwardRight(5, 0)
                    .closeBothClaws()
                    .sleepFor(321)
                    .backwardLeft(3, 3)
                    .raiseArm(12, true)  // 15
                    .flipElbow()
                    .sleepFor(333)
                    .restElbow()
                    .backwardLeft(20, 19)
                    .backward(52)
                    .backwardRight(30, 30)
                    .raiseArm(175, true)  // 190
                    .openBothClaws()
                    .sleepFor(111)
                    .lowerArm(170)  // 20
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
            drive.followMovementSequence(white);
//            drive.followMovementSequence(park);
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