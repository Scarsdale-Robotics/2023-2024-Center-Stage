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

    }

    /**
     * GROUP 1
     * aligns such that a ray representing a robot's camera direction is (anti-)parallel to a ray extending "outwards" from the center of a given AprilTag
     * @param tagID the id of the AprilTag from the 36h11 family to align with
     */
    public void alignParallelWithAprilTag(int tagID) {

    }
}
