package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;

public class SubsystemInitializer {
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;
    public SubsystemInitializer(HardwareRobot robot, boolean isRedTeam, LinearOpMode opMode, Telemetry telemetry) {
        inDep = new InDepSubsystem(
                robot.arm1,
                robot.arm2,
                robot.elbow, robot.wrist, robot.leftClaw, robot.rightClaw,
                opMode
        );
        drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                inDep,
                opMode
        );
        cvFront = new CVSubsystem(robot.frontCam,
                robot.frontCamName,
                drive,
                telemetry,
                isRedTeam,
                opMode);
        cvBack = new CVSubsystem(robot.backCam,
                robot.backCamName,
                drive,
                telemetry,
                isRedTeam,
                opMode);
        MovementThread.initSubsystems(drive, inDep, cvFront, cvBack, opMode);
    }

    public InDepSubsystem getInDep() {
        return inDep;
    }
    public DriveSubsystem getDrive() {
        return drive;
    }
    public CVSubsystem getCVFront() {
        return cvFront;
    }
    public CVSubsystem getCVBack() {
        return cvFront;
    }
}
