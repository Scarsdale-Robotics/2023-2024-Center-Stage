package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

import java.util.ArrayList;

public class AutoUtility {
    private final int SAMPLE_COUNT = 200;
    private final long SAMPLE_WAIT_MILLISECONDS = 25;
    private CVSubsystem cv;
    private DriveSubsystem drive;
    private LinearOpMode opMode;
    private InDepSubsystem inDep;
    public AutoUtility(CVSubsystem cv, DriveSubsystem drive, InDepSubsystem inDep, LinearOpMode opMode) {
        this.cv = cv;
        this.drive = drive;
        this.inDep = inDep;
        this.opMode = opMode;
    }

    public int getPropLocation(Telemetry telemetry) throws InterruptedException {
        int propLocation = -1;
        ArrayList<Integer> samples = new ArrayList<>();
        Thread.sleep(1000);
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            if (!opMode.opModeIsActive()) break;
            propLocation = cv.getTeamPropLocation();
            if (-1 < propLocation && propLocation < 3) samples.add(propLocation);
            Thread.sleep(SAMPLE_WAIT_MILLISECONDS);
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
        return propLocation;
    }

    public int placePurplePixel(int propLocation, boolean isRedTeam, Telemetry telemetry) {
        if (propLocation == 0) {

        } else if (propLocation == 1) {

        } else if (propLocation == 2) {

        }
        inDep.open(); // open claw to place the pixel
        return propLocation;
    }
}
