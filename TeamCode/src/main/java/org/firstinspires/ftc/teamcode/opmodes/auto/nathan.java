package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

import java.util.ArrayList;

@Autonomous(name = "temp nathan")
public class nathan extends LinearOpMode {
    private final int SAMPLE_COUNT = 200;

    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        CVSubsystem cv = new CVSubsystem(robot.camera,
                robot.cameraName,drive, telemetry, true, this);


        waitForStart();

        telemetry.addData("team prop loc: ", "---");
        telemetry.update();
        while (opModeIsActive()) {
            int propLocation = -1;
            ArrayList<Integer> samples = new ArrayList<>();
            for (int i = 0; i < SAMPLE_COUNT; i++) {
                if (!opModeIsActive()) break;
                propLocation = cv.getTeamPropLocation();
                if (-1 < propLocation && propLocation < 3) samples.add(propLocation);
                Thread.sleep(10);
            }
            // parsing early misreads
            if (samples.size()>1) samples.remove(0);
            int[] locations = new int[3];
            for (int i : samples) locations[i]++;
            int max = 0;
            for (int i = 0; i < 3; i++) {
                if (locations[i] > max) {
                    max = locations[i];
                    propLocation = i;
                }
            }
            telemetry.addData("",samples);
            telemetry.addData("loc: ",propLocation);
            telemetry.addData("locs0: ", locations[0]);
            telemetry.addData("locs1: ", locations[1]);
            telemetry.addData("locs2: ", locations[2]);
            telemetry.update();
        }

    }
}
