package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

public class CVSubsystem {

    OpenCvCamera camera;

    public final int LOCATION_LEFT   =  0;
    public final int LOCATION_CENTER =  1;
    public final int LOCATION_RIGHT  =  2;
    public final int NO_LOCATION     = -1;

    public final double NO_ROTATIONAL_OFFSET = -50000.0;
    public final double NO_SIZE = -50000.0;
    public final double ERROR   =  3.0;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public CVSubsystem() {
        // create AprilTagProcessor and VisionPortal
        initAprilTag();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        // to modify, look for the specs in ConceptAprilTag.java:
        //.setDrawAxes(false)
        //.setDrawCubeProjection(false)
        //.setDrawTagOutline(true)
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the built-in RC phone camera
        builder.setCamera(BuiltinCameraDirection.BACK);
        builder.setAutoStopLiveView(false); // keep camera on when not processing

        builder.setCameraResolution(new Size(1280, 960)); // android.util
        //default 640 480

        // Choose a camera resolution. Not all cameras support all resolutions.

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return whether the AprilTag is left, center, or right in the camera view
     */

    public int getAprilTagLocation(int tagID) {

        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                if (detection.id == tagID) {
                    if (detection.ftcPose.z < -ERROR) return LOCATION_LEFT;
                    else if (detection.ftcPose.z >  ERROR) return LOCATION_RIGHT;
                    else return LOCATION_CENTER;
                }
            }
        }

        return NO_LOCATION;
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return a double representing the amount the robot should turn to be "parallel" to the AprilTag
     */
    public double getAprilTagRotationalOffset(int tagID) { // return yaw
        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double rotationalOffset = NO_ROTATIONAL_OFFSET;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    rotationalOffset = detection.ftcPose.yaw;
                }
            }
        }

        return rotationalOffset; // TEMPORARY
    }

    //get the angle
    public double getAprilTagSize(int tagID) {
        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagSize = NO_SIZE;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    //tagSize = detection.ftcPose.;
                }
            }
        }
        //visionPortal.stopStreaming();

        return tagSize; // TEMPORARY
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
