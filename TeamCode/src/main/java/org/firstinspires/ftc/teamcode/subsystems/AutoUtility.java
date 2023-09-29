package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.core.DriveSubsystem;

public class AutoUtility {
    private DriveSubsystem drive;
    private CVSubsystem cv;
    public AutoUtility(DriveSubsystem drive, CVSubsystem cv) {
        this.drive = drive;
        this.cv = cv;
    }

    /**
     * GROUP 1
     * get a given AprilTag's position and moves in front of it
     * use alignParallelWithAprilTag() and getAprilTagPosition()
     *
     * @param tagID the id of the AprilTag from the 36h11 family to move to
     */
    public void moveToAprilTag(int tagID) {
        int aprilTagLocation = cv.getAprilTagLocation(0);  // temp id
        while (aprilTagLocation == cv.LOCATION_LEFT) {
            drive.driveFieldCentric(-1, 0, 0);
            aprilTagLocation = cv.getAprilTagLocation(0);
        }
        while (aprilTagLocation == cv.LOCATION_RIGHT) {
            drive.driveFieldCentric(1, 0, 0);
            aprilTagLocation = cv.getAprilTagLocation(0);
        }
        while (cv.getAprilTagSize(0) < 1) {
            drive.driveFieldCentric(0, 1, 0);
        }
    }
    /**
     * GROUP 1
     * aligns such that a ray representing a robot's camera direction is (anti-)parallel to a ray extending "outwards" from the center of a given AprilTag
     * @param tagID the id of the AprilTag from the 36h11 family to align with
     */
    public void alignParallelWithAprilTag(int tagID) {

    }
}
