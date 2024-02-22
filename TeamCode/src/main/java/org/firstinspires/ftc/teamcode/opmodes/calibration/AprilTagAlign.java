package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous
@Config
public class AprilTagAlign extends LinearOpMode {
    public static double turnOffset = 16.5;
    public static double yOffset = 10;
    public static double xOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        waitForStart();
        MovementSequence main = new MovementSequenceBuilder()
//                .turnRight(16.5)
//                .left(5)
//                .forward(10)
                .alignWithAprilTagPos(9, yOffset, xOffset, turnOffset)
                .alignWithAprilTagParRot(9, turnOffset)
                .build();
        robot.getDrive().followMovementSequence(main);
    }
}
