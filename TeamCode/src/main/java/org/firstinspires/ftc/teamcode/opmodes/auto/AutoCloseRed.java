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

@Autonomous(name = "Auto Close Blue")
@Config
public class AutoCloseBlue extends LinearOpMode {
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
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                //stuff
                .build();
        int propLocation = 0;
        drive.followMovementSequence(initCV);
        if (!useString)
            propLocation = robot.getCV().getPropLocation();

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                approachWhite = new MovementSequenceBuilder().build(),
                placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forwardRight(19.8,11.16)
                    .openRightClaw() // drop purple pixel
                    .sleepFor(500)
                    .backward(5)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .raiseArm(30)
                    .turnRight(90, true)
                    .forwardLeft(24,10)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .backward(5)
                    .turnRight(180)
                    .left(20)
                    .backward(9)
                    .lowerArm(20)
                    .lowerArm(10)
                    .forward(2)
                    .build();

        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder().closeBothClaws()
                    .forward(28)
                    .openRightClaw() // drop purple pixel
                    .sleepFor(500)
                    .backward(4)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .raiseArm(30)
                    .turnRight(90, true)
                    .forwardLeft(33,4)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .backward(5)
                    .turnLeft(180)
                    .lowerArm(20)
                    .left(24, true)
                    .backward(18)
                    .lowerArm(10)
                    .forward(2)
                    .build();

        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()

                    .build();
            placeYellow = new MovementSequenceBuilder()

                    .build();
            park = new MovementSequenceBuilder()

                    .build();
        }

        if (useString) {
            MovementSequence seq = new MovementSequenceBuilder()
                    .append(sequence)
                    .build();
            drive.followMovementSequence(seq);
        }

        if (!useString) {
            // perform the actual movements here in sequence
            drive.followMovementSequence(placePurple);
            drive.followMovementSequence(placeYellow);
            //        for (int i=0;i<WHITE_REPS;i++)
            //        {
            //            drive.followMovementSequence(approachWhite);
            //            drive.followMovementSequence(placeWhite);
            //        }
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