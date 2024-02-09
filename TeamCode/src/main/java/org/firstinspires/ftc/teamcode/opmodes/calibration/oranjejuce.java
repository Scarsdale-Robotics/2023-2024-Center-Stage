package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "oranje juce")
@Config
public class oranjejuce extends LinearOpMode {
    private HardwareRobot hardwareRobot;
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    private EndgameSubsystem endgame;
    private CVSubsystem cvSubsystem;

    // Configuration constants
    public static double leftPosOpen = 0.6;
    // Other configuration constants...

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareRobot = new HardwareRobot(hardwareMap);
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                this
        );
        inDep = new InDepSubsystem(
                hardwareRobot.arm1,
                hardwareRobot.arm2,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                this
        );
        endgame = new EndgameSubsystem(hardwareRobot.drone);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cvSubsystem = new CVSubsystem(hardwareRobot.backCamName, hardwareRobot.frontCamName, drive, telemetry, true, this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened.");
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Telemetry data from CVSubsystem or other operations
            telemetry.addData("Status", "Running");
            telemetry.addData("Color Detection", "Press 'A' to detect colors");

            if (gamepad1.a) {
                cvSubsystem.identifyColorRangesInLocations();
            }

            telemetry.update();
        }

        camera.stopStreaming();
    }
}