package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "[2+0] Auto Close Blue Whiteless")
@Config
public class AutoCloseBlueWhiteless extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private DriveSubsystem drive;
    public static String sequence = "";
    public static boolean useString = false;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
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

        double turnOffset = 16.5;

        MovementSequence purpleYellow = new MovementSequenceBuilder().build(),
                white = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            purpleYellow = new MovementSequenceBuilder()
                    // (16.75,63.5,-90)
                    .forwardLeft(29, 6.5+15)
                    .raiseArm(3.5, true)  // 3.5
                    .turnRight(90)
                    .openRightClaw()
                    .sleepFor(321)
                    .backwardRight(17-15+1, 10)
                    .raiseArm(211.5, true)  // 215
                    .backward(4)
                    .openLeftClaw()
                    .sleepFor(333)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(8, 32.5)
                    .lowerArm(150, true)  // 65
                    .forwardRight(15.5+72.5+2.5, 1)
                    .lowerArm(57, true)  // 12
                    .closeBothClaws()  // white intake
                    .sleepFor(500)
                    .backwardRight(4, 0)
                    .raiseArm(22, true)  // 30
                    .closeBothClaws()  // high number of closeBothClaws()
                    .flipElbow()
                    .sleepFor(321)
                    .closeBothClaws()  // keep them
                    .restElbow()
                    .backwardLeft(80-3, 0)
                    .backwardRight(22, 31)
                    .raiseArm(160, true)  // 190
                    .openBothClaws()  // white deposit
                    .sleepFor(321)
                    .lowerArm(170)  // 20
                    .forward(3)
                    .lowerArm(20, true)  // 0
                    .backward(3)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(5)
                    .right(18)
                    .lowerArm(195, true)  // 20
                    .backward(15)
                    .forward(5)
                    .lowerArm(20, true)
                    .build();
        } else if (propLocation == 1) {
            purpleYellow = new MovementSequenceBuilder()
                    //(16.75,63.5,-90)
                    .forwardLeft(39.5, 15)
                    .raiseArm(10, true)  // 10
                    .turnRight(90)
                    .openRightClaw()
                    .sleepFor(250)
                    .backwardLeft(25-11.5, 4-14.5-1-2.5)
                    .raiseArm(205, true)  // 215
                    .openLeftClaw()
                    .sleepFor(333)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(14, 27)
                    .lowerArm(160, true)  // 55
                    .forwardRight(86, 0.5)
                    .lowerArm(47, true) // 8
                    .closeBothClaws()
                    .sleepFor(500)
                    .backwardRight(3, 0)
                    .raiseArm(12, true)  // 20
                    .closeBothClaws()
                    .flipElbow()
                    .sleepFor(321)
                    .closeBothClaws()
                    .restElbow()
                    .backwardLeft(80, 0)
                    .backwardRight(14, 25)
                    .raiseArm(170, true)  // 190
                    .backward(5)
                    .openBothClaws()
                    .sleepFor(500)
                    .lowerArm(175)  // 15
                    .forward(3)
                    .lowerArm(15, true)  // 0
                    .backward(3)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(5)
                    .right(24)
                    .lowerArm(195, true)  // 20
                    .backward(15)
                    .forward(5)
                    .lowerArm(20, true)
                    .build();
        } else if (propLocation == 2) {
            purpleYellow = new MovementSequenceBuilder()
                    // 43 30 0
                    // 36 6
                    // (16.75,63.5,-90)
                    .forwardRight(29.5, 1)
                    .raiseArm(3.5, true)  // 3.5
                    .turnRight(90)
                    .forward(2)
                    .openRightClaw()
                    .sleepFor(333)
                    .backwardLeft(30.5, 6)
                    .raiseArm(211.5, true)  // 215
//                    .backward(3)
                    .openLeftClaw()
                    .sleepFor(333)
                    .build();
            white = new MovementSequenceBuilder()
                    .forwardLeft(9.5, 23-1.75-3.5+1.5)
                    .lowerArm(207, true)  // 8
                    .forwardRight(72.5+15.5+6, 0.15)
                    .closeBothClaws()  // white intake
                    .sleepFor(555)
                    .backwardRight(4, 0)
                    .raiseArm(17, true)  // 25
                    .closeBothClaws()
                    .flipElbow()
                    .sleepFor(321)
                    .closeBothClaws()
                    .restElbow()
                    .backwardLeft(79, 0)
                    .backwardRight(19, 29-1.75-1+1.5)
                    .raiseArm(165, true)  // 190
                    .backward(3)
                    .openBothClaws()  // white deposit
                    .sleepFor(333)
                    .lowerArm(175)  // 15
                    .forward(3)
                    .lowerArm(15, true)  // 0
                    .backward(3)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(5)
                    .right(30)
                    .lowerArm(195, true)  // 20
                    .backward(15)
                    .forward(5)
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