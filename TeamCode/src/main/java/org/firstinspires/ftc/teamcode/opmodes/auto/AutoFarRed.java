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

@Autonomous(name = "Auto Far Red")
@Config
public class AutoFarRed extends LinearOpMode {
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
                .sleepFor(1000)
                .build();

        int propLocation = 0;

        if (!useString) {
            drive.followMovementSequence(initCV);
            propLocation = robot.getCV().getPropLocation();
        }
        robot.getCV().disablePropProcessor();

        MovementSequence main = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            main = new MovementSequenceBuilder()
                    .forwardRight(37.5, 2.75)
                    .turnLeft(90)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(4)
                    .backwardLeft(1, 10)
                    .forwardLeft(28, 1)
                    .raiseArm(10, true)
                    .closeRightClaw()
                    .sleepFor(150)
                    .backwardLeft(32, 23)
                    .backward(41)
                    .raiseArm(10, true)
                    .backwardRight(33, 30)
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(3)
                    .openBothClaws()
                    .sleepFor(150)
                    .forward(3)
                    .forwardLeft(33, 30)
                    .restElbow()
                    .closeLeftClaw()
                    .lowerArm(210, true)
                    .forward(41)
                    .forwardRight(32, 23)
                    .lowerArm(10, true)
                    .closeRightClaw()
                    .sleepFor(500)
                    .backwardLeft(32, 23)
                    .backward(41)
                    .raiseArm(10, true)
                    .backwardRight(33, 18)
                    .flipElbow()
                    .raiseArm(220, true)
                    .backward(3)
                    .openRightClaw()
                    .sleepFor(150)
                    .forward(7)
                    .right(29)
                    .restElbow()
                    .lowerArm(220, true)
                    .openLeftClaw()
                    .build();
        } else if (propLocation == 1) {
            main = new MovementSequenceBuilder()
                    .forwardRight(29.5, 1.75)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(4)
                    .turnLeft(90)
                    .forward(21)
                    .raiseArm(10, true)
                    .closeRightClaw()
                    .sleepFor(150)
                    .backwardLeft(30, 22)
                    .backward(49)
                    .raiseArm(10, true)
                    .backwardRight(29, 23)
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(3)
                    .openLeftClaw()
                    .sleepFor(150)
                    .openRightClaw()
                    .sleepFor(150)
                    .forward(3)
                    .forwardLeft(29, 23)
                    .restElbow()
                    .closeLeftClaw()
                    .lowerArm(210, true)
                    .forward(49)
                    .forwardRight(30, 22)
                    .lowerArm(10, true)
                    .closeRightClaw()
                    .sleepFor(500)
                    .backwardLeft(30, 22)
                    .raiseArm(10, true)
                    .backward(49)
                    .raiseArm(10, true)
                    .backwardRight(30, 30)
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(2)
                    .openRightClaw()
                    .sleepFor(150)
                    .forward(2)
                    .right(18)
                    .closeRightClaw()
                    .restElbow()
                    .lowerArm(220, true)
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