package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.core.DriveSubsystem;

public class AutoUtility {
    private DriveSubsystem drive;
    private CVSubsystem cv;
    private SpeedCoefficients speed;
    public AutoUtility(DriveSubsystem drive, CVSubsystem cv, SpeedCoefficients speed) {
        this.drive = drive;
        this.cv = cv;
        this.speed = speed;
    }

    /**
     * GROUP 1
     * get a given AprilTag's position and moves in front of it
     * use alignParallelWithAprilTag() and getAprilTagPosition()
     *
     * @param tagID the id of the AprilTag from the 36h11 family to move to
     */
    public void moveToAprilTag(int tagID) {
        double DISTANCE_THRESHOLD = 1;  // distance from backboard to stop at

        alignParallelWithAprilTag(tagID);
        while (cv.getAprilTagDistance(tagID) > DISTANCE_THRESHOLD) {
            int aprilTagLocation = cv.getAprilTagLocation(tagID);
            switch (aprilTagLocation) {
                case 0:
                    // left location
                    drive.driveFieldCentric(speed.getStrafeSpeed(), 0, 0);
                    break;
                case 2:
                    // right location
                    drive.driveFieldCentric(-speed.getStrafeSpeed(), 0, 0);
                    break;
                default:
                    // center location
                    drive.driveFieldCentric(0, speed.getForwardSpeed(), 0);
                    break;
            }
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
