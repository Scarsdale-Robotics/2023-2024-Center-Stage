package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;

public class RobotSystem {
    private HardwareRobot hardwareRobot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cv;
    private EndgameSubsystem endgame;

    public RobotSystem(HardwareMap hardwareMap, boolean isRedTeam, LinearOpMode opMode, Telemetry telemetry) {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.arm1,
                hardwareRobot.arm2,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                opMode,
                multipleTelemetry
        );
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                opMode,
                multipleTelemetry
        );
        cv = new CVSubsystem(
                hardwareRobot.frontCamName,
                hardwareRobot.backCamName,
                drive,
                telemetry,
                isRedTeam,
                opMode);
        endgame = new EndgameSubsystem(hardwareRobot.drone);
        MovementThread.initSubsystems(drive, inDep, cv, opMode);
        drive.resetIMU();
    }

    public HardwareRobot getHardwareRobot() {
        return hardwareRobot;
    }
    public AdafruitBNO055IMU getIMU() {
        return hardwareRobot.imu;
    }
    public InDepSubsystem getInDep() {
        return inDep;
    }
    public DriveSubsystem getDrive() {
        return drive;
    }
    public CVSubsystem getCV() {
        return cv;
    }
    public EndgameSubsystem getEndgame() {
        return endgame;
    }
}
