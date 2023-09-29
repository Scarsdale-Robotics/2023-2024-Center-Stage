package org.firstinspires.ftc.teamcode.subsystems;

public class CVSubsystem {
    public final int LOCATION_LEFT = 0;
    public final int LOCATION_CENTER = 1;
    public final int LOCATION_RIGHT = 2;

    public CVSubsystem() {
        // create AprilTagProcessor and VisionPortal
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return whether the AprilTag is left, center, or right in the camera view
     */
    public int getAprilTagLocation(int tagID) {
        return 0; // TEMPORARY
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return a double representing the amount the robot should turn to be "parallel" to the AprilTag
     */
    public double getAprilTagRotationalOffset(int tagID) {
        return 0; // TEMPORARY
    }

    public double getAprilTagDistance(int tagID) {
        return 0; // TEMPORARY
    }

    /**
     * GROUP 2
     * @param isRedTeam true if our alliance team is red, false otherwise
     * @return whether the AprilTag is left, center, or right in the camera view
     */
    public int getTeamPropLocation(boolean isRedTeam) {
        return 0; // TEMPORARY
    }

    /**
     * GROUP 2
     * @return true if the robot is in front of a piece of tape approximately perpendicular to the camera view, false otherwise
     */
    public boolean isRobotBeforeTape(boolean isRedTeam) {
        return false; // TEMPORARY
    }


}
