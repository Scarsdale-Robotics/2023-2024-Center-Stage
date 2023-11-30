package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

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

    public int placePurplePixelBlue(int moveOffset, boolean isCloseBlue, Telemetry telemetry) throws InterruptedException {
        // DO NOT CHANGE THIS METHOD
        // moveOffset param influences how much we move forward before turning
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


        //drive.driveByEncoder(0, 0, 0.5, (isCloseBlue ? 880*4 : 0));  // turn left 360 degrees

        if (isCloseBlue){
            drive.driveByEncoder(0.3, 0, 0, 260); // strafe left
            drive.driveByEncoder(0, -0.3, 0, 350); // move forward
        }
        if (propLocation == 0) {
            drive.driveByEncoder(0, -0.3, 0, 720 + moveOffset); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 888 + (isCloseBlue ? -45         : 18));  // turn left 90 degrees
            drive.driveByEncoder(0, -0.3 * (isCloseBlue ? -1 : 1), 0, 50 + (isCloseBlue ? 50 : 30)); // moving forward (or backward) to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(-0.3, 0, 0, 300 + (isCloseBlue ? -50 : 0)); // strafe right to place pixel correctly
        } else if (propLocation == 1) {
            drive.driveByEncoder(0, -0.3, 0, (isCloseBlue ? 835 + moveOffset : 800 + moveOffset)); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0.3, 0, 55); // move backward to not hit pixel on turn
        } else if (propLocation == 2) {
            drive.driveByEncoder(0, -0.3, 0, 800 + moveOffset); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 915 + (isCloseBlue ? -45 : -60));  // turn right
            //drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            //drive.driveByEncoder(0, 0.3, 0, 1); // brake
            if (!isCloseBlue) drive.driveByEncoder(0, 0.3, 0, 160 + (isCloseBlue ? 0 : 50)); // moving back to center
            drive.driveByEncoder(0, -0.3, 0, 10); // brake
            drive.driveByEncoder(0.3, 0, 0, 200); // strafe left
        }
        inDep.open(); // open claw to place the pixel
        return propLocation;
    }

    public int placePurplePixelRed(int moveOffset, boolean isCloseRed, Telemetry telemetry) throws InterruptedException {
        // DO NOT CHANGE THIS METHOD
        // moveOffset param influences how much we move forward before turning
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


        //drive.driveByEncoder(0, 0, 0.5, (isCloseBlue ? 880*4 : 0));  // turn left 360 degrees

        if (!isCloseRed){
            drive.driveByEncoder(0.3, 0, 0, 260-25); // strafe left
            drive.driveByEncoder(0, -0.3, 0, 350); // move forward
        }
        if (propLocation == 2) {
            drive.driveByEncoder(0, -0.3, 0, 700 + moveOffset + (isCloseRed ? 100 : 0)); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 888 + (isCloseRed ? -30 : 0));  // turn right 90 degrees
            drive.driveByEncoder(0, -0.3 * (isCloseRed ? -1 : -1), 0, 50 + (isCloseRed ? 100 : -50)); // moving forward (or backward) to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0.3, 0, 0, (isCloseRed ? 0 : 300)); // strafe left to place pixel correctly
        } else if (propLocation == 1) {
            drive.driveByEncoder(0, -0.3, 0, (isCloseRed ? 669.69 + moveOffset : 779 + moveOffset)); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
        } else if (propLocation == 0) {
            drive.driveByEncoder(0, -0.3, 0, 800 + moveOffset); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 915 + (isCloseRed ? -27 : -50));  // turn left
            drive.driveByEncoder(0, -0.3 * (isCloseRed ? 1 : -1), 0, (isCloseRed ? 33 : 155)); // moving back/approach
            drive.driveByEncoder(0, -0.3, 0, 10); // brake
            drive.driveByEncoder(-0.3, 0, 0, (isCloseRed ? 0 : 200)); // strafe right
        }
        inDep.open(); // open claw to place the pixel
        return propLocation;
    }
}
