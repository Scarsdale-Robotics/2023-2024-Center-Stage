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

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                approachWhite = new MovementSequenceBuilder().build(),
                placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()  // -40.75 -63.5
                    .forwardLeft(17.5, 7.25)  // -48 -46
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(8)  // -48 -54
                    .backwardRight(6, 10)  // -38 -60
                    .turnLeft(90)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    // arm needs 220
                    .backward(53)  // 15 -60
                    .raiseArm(20, true)
                    .backwardRight(29, 30)  // 44 -30
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(5)  // 49 -30
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)  // 41 -30
                    .right(18)  // 41 -12
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(20)  // 61 -12
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)  // 59 -12
                    .build();

        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()  // -40.75 -63.5
                    .forwardRight(28.5, 2.75)  // -38 -35
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(25)  // -38 -60
                    .turnLeft(90)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    // arm needs 220
                    .backward(53)  // 15 -60
//                    .raiseArm(50, true) // bugged
                    .raiseArm(20, true)
                    .backwardRight(29, 23)  // 44 -37
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(5)  // 49 -37
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)  // 41 -37
                    .right(25)  // 41 -12
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(20)  // 61 -12
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)  // 59 -12
                    .build();

        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()  // -40.75 -63.5
                    .forwardRight(28.5, 5.75)  // -35 -35
                    .turnRight(90)
                    .openRightClaw()
                    .sleepFor(150)
                    .backward(8)  // -43 -35
                    .forwardRight(5, 25)  // -38 -60
                    .turnLeft(180)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    // arm needs 220
                    .backward(53)  // 15 -60
                    .raiseArm(20, true)
                    .backwardRight(29, 15)  // 44 -45
                    .flipElbow()
                    .raiseArm(200, true)
                    .backward(5)  // 49 -45
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(8)  // 41 -45
                    .right(33)  // 41 -12
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(20)  // 61 -12
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)  // 59 -12
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