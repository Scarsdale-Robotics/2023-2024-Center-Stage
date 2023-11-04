package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PropDetectionPipeline;

import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelDetectionPipeline;

import java.util.List;

public class CVSubsystem extends SubsystemBase {
    private OpenCvCamera camera;
    private DriveSubsystem drive;

    private final int LOCATION_LEFT   =  0;
    private final int LOCATION_CENTER =  1;
    private final int LOCATION_RIGHT  =  2;
    private final int NO_LOCATION     = -1;

    private final double NO_ROTATIONAL_OFFSET = -50000.0;
    private final double NO_DISTANCE = -50000.0;
    private final double ERROR   =  3.0;
    private final double ERROR_ALIGNMENT = 0.5;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public CVSubsystem(OpenCvCamera camera, DriveSubsystem drive) {
        this.camera = camera;
        this.drive = drive;
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

        return rotationalOffset;
    }

    public double getAprilTagDistance(int tagID) {
        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = NO_DISTANCE;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    tagDistance = detection.ftcPose.range;
                }
            }
        }
        //visionPortal.stopStreaming();

        return tagDistance; // TEMPORARY
    }

    /**
     * GROUP 2
     * @param isRedTeam true if our alliance team is red, false otherwise
     * @return whether the AprilTag is left, center, or right in the camera view
     */
    public int getTeamPropLocation(boolean isRedTeam) {
        propPipeline = new PropDetectionPipeline(telemetry, isRedTeam);
        camera.setPipeline(propPipeline);
        return propPipeline.getPosition();
    }
    public int getPixelHorizontalOffset() {
        PixelDetectionPipeline p = new PixelDetectionPipeline();
        camera.setPipeline(p);
        return p.getCenterOffset();
    }

    /**
     * GROUP 2
     * @return true if the robot is in front of a piece of tape approximately perpendicular to the camera view, false otherwise
     */
    public boolean isRobotBeforeTape(boolean isRedTeam) {
        return false; // TEMPORARY
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
        while (getAprilTagDistance(tagID) > DISTANCE_THRESHOLD) {
            int aprilTagLocation = getAprilTagLocation(tagID);
            switch (aprilTagLocation) {
                case 0:
                    // left location
                    drive.driveFieldCentric(1, 0, 0);
                    break;
                case 2:
                    // right location
                    drive.driveFieldCentric(-1, 0, 0);
                    break;
                default:
                    // center location
                    drive.driveFieldCentric(0, 1, 0);
                    break;
            }
        }
    }

    /**
     * aligns such that a ray representing a robot's camera direction is (anti-)parallel to a ray extending "outwards" from the center of a given AprilTag
     * @param tagID the id of the AprilTag from the 36h11 family to align with
     */
    public void alignParallelWithAprilTag(int tagID) {
        double rotOff = getAprilTagRotationalOffset(tagID);
        while (Math.abs(rotOff) > ERROR_ALIGNMENT) {
            drive.driveFieldCentric(0, 0, rotOff * 1); // times some scaling factor (temporarily at 1)
        }
    }
}
