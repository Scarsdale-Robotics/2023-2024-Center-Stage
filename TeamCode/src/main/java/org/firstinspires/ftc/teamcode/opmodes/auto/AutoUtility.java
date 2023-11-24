package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

import java.util.ArrayList;

public class AutoUtility {
    private final int SAMPLE_COUNT = 5;
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

    public int placePurplePixelBlue(int moveOffset, boolean isCloseBlue) throws InterruptedException {
        // DO NOT CHANGE THIS METHOD
        // moveOffset param influences how much we move forward before turning
        int propLocation = -1;
        ArrayList<Integer> samples = new ArrayList<>();
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            if (!opMode.opModeIsActive()) break;
            propLocation = cv.getTeamPropLocation();
            if (-1 < propLocation && propLocation < 3) samples.add(propLocation);
            Thread.sleep(1000);
        }
        propLocation = 0;
        int[] locations = new int[3];
        for (int i : samples) locations[i]++;
        for (int i = 0; i < locations.length; i++) {
            propLocation = locations[i] > locations[propLocation] ? i : propLocation;
        }

        //drive.driveByEncoder(0, 0, 0.5, (isCloseBlue ? 880*4 : 0));  // turn left 360 degrees
        if (propLocation == 0) {
            drive.driveByEncoder(0, -0.3, 0, 700 + moveOffset); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 888 + (isCloseBlue ? -30 : 0));  // turn left 90 degrees
            drive.driveByEncoder(0, -0.3 * (isCloseBlue ? -1 : 1), 0, 50 + (isCloseBlue ? 15 : 0)); // moving forward (or backward) to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(-0.3, 0, 0, 300 + (isCloseBlue ? -50 : 0)); // strafe right to place pixel correctly
        } else if (propLocation == 1) {
            drive.driveByEncoder(0, -0.3, 0, (isCloseBlue ? 850 + moveOffset : 800 + moveOffset)); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
        } else if (propLocation == 2) {
            drive.driveByEncoder(0, -0.3, 0, 800 + moveOffset); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 915 + (isCloseBlue ? -45 : 0));  // turn right
            //drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            //drive.driveByEncoder(0, 0.3, 0, 1); // brake
            if (!isCloseBlue) drive.driveByEncoder(0, 0.3, 0, 100); // moving back to center
            drive.driveByEncoder(0, -0.3, 0, 10); // brake
            drive.driveByEncoder(0.3, 0, 0, 200); // strafe left
        }
        inDep.open(); // open claw to place the pixel
        return propLocation;
    }
}
