package org.firstinspires.ftc.teamcode.subsystems.movement;

public class AprilTagValues {
    public int tagID;
    public double turnOffset;
    public double xOffset;
    public double yOffset;

    AprilTagValues() {
        this(0, 0, 0, 0);
    }

    AprilTagValues(int tagID, double turnOffset, double xOffset, double yOffset) {
        this.tagID = tagID;
        this.turnOffset = turnOffset;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }
}
