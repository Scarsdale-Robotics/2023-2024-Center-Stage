package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;

public class RobotSystem {
    private HardwareRobot hardwareRobot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;

    public RobotSystem(HardwareMap hardwareMap, boolean isRedTeam, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.arm1,
                hardwareRobot.arm2,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                opMode
        );
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                opMode
        );
//        cvFront = new CVSubsystem(hardwareRobot.frontCam,
//                hardwareRobot.frontCamName,
//                drive,
//                telemetry,
//                isRedTeam,
//                opMode);
//        cvBack = new CVSubsystem(hardwareRobot.backCam,
//                hardwareRobot.backCamName,
//                drive,
//                telemetry,
//                isRedTeam,
//                opMode);
        MovementThread.initSubsystems(drive, inDep, cvFront, cvBack, opMode);
    }

    public HardwareRobot getHardwareRobot() {
        return hardwareRobot;
    }
    public IMU getIMU() {
        return hardwareRobot.imu;
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
