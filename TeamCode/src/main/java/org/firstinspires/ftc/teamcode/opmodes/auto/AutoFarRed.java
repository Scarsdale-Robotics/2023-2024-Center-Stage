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
        robot.getInDep().autoInit();
        robot.getCV().autoExposure();
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
            main = new MovementSequenceBuilder()// START POSE (-40.75,-63.5,90)
                    .forwardLeft(26.5-12, 7.25+2)
                    .raiseArm(3.5, true)  // 3.5
                    .openRightClaw()
                    .sleepFor(300)
                    .backwardRight(23-14, 16)
                    .turnLeft(90)
                    .backward(39+4.5)
                    .raiseArm(21.5, true)  // 25
                    .backwardRight(34, 30+3)
                    .raiseArm(185, true)  // 210
                    .openLeftClaw()
                    .sleepFor(300)
                    .forward(2)
                    .right(16)
                    .lowerArm(190, true)  // 20
                    .backward(12)
                    .forward(8)
                    .lowerArm(20, true)  // 0
                    .build();
        } else if (propLocation == 1) {
            main = new MovementSequenceBuilder()// START POSE (-40.75,-63.5,90)
                    .forwardLeft(26.5-3, 7.25-7.25)
                    .raiseArm(3.5, true)  // 3.5
                    .openRightClaw()
                    .sleepFor(300)
                    .backwardRight(18, 3)
                    .turnLeft(90)
                    .backward(39+9+5)
                    .backwardRight(27, 27)
                    .raiseArm(206.5, true)  // 210
                    .openLeftClaw()
                    .sleepFor(300)
                    .forward(2)
                    .right(24)
                    .lowerArm(190, true)  // 20
                    .backward(12)
                    .forward(8)
                    .lowerArm(20, true)  // 0
                    .build();
        } else if (propLocation == 2) {
            main = new MovementSequenceBuilder()// START POSE (-40.75,-63.5,90)
                    .forwardRight(27.5, 7.75-4.75)
                    .raiseArm(3.5, true)  // 3.5
                    .turnRight(90)
                    .forwardLeft(0, 2)
                    .openRightClaw()
                    .sleepFor(300)
                    .backwardRight(3, 1)
                    .backwardRight(7, 24)
                    .turnLeft(180)
                    .backward(66+4.75)
                    .backwardRight(19, 16)
                    .raiseArm(206.5, true)  // 210
                    .openLeftClaw()
                    .sleepFor(300)
                    .forward(2)
                    .right(33)
                    .lowerArm(190, true)  // 20
                    .backward(15)
                    .forward(10)
                    .lowerArm(20, true)  // 0
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