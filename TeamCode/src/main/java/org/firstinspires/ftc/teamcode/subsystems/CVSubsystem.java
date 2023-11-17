package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.TapeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PropDetectionPipeline;

import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.List;

public class CVSubsystem extends SubsystemBase {
    private OpenCvCamera camera;
    private DriveSubsystem drive;

    private final int LOCATION_LEFT   =  0;
    private final int LOCATION_CENTER =  1;
    private final int LOCATION_RIGHT  =  2;
    private final int NO_LOCATION     = -1;
    private Telemetry telemetry;

    private final double NO_ROTATIONAL_OFFSET = -50000.0;
    private final double NO_DISTANCE = -50000.0;
    private final double ERROR   =  3.0;
    private final double ERROR_ALIGNMENT = 0.5;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final WebcamName cameraName;
    private TapeDetectionPipeline tdp;
    private PropDetectionPipeline propPipeline;
    private PixelDetectionPipeline pixelPipeline;
    public String poo = "Tho";
    PropDetectionPipeline pdp = new PropDetectionPipeline();
    public CVSubsystem(OpenCvCamera camera, WebcamName cameraName, DriveSubsystem drive, Telemetry telemetry, boolean isRedTeam) {
//        tdp = new TapeDetectionPipeline();
//        camera.setPipeline(tdp);
        this.camera = camera;
        this.drive = drive;
        this.telemetry = telemetry;
        this.cameraName = cameraName;
        // create AprilTagProcessor and VisionPortal
        initAprilTag();

        pixelPipeline = new PixelDetectionPipeline();
//        propPipeline = new PropDetectionPipeline(isRedTeam, telemetry);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.setPipeline(pixelPipeline);
////                camera.setPipeline(propPipeline);
//                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//                poo = "Nathan messed up !!!1!";
//            }
//        });
    }

    private void initAprilTag() {
        poo = "ick";
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
//        builder.setCamera(BuiltinCameraDirection.BACK);
        builder.setCamera(cameraName);
        builder.setAutoStopLiveView(false); // keep camera on when not processing

        builder.setCameraResolution(new Size(640, 480)); // android.util

        builder.addProcessor(pdp);

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
        poo = "nu";
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
     * @return whether the AprilTag is left, center, or right in the camera view
     */
    public int getTeamPropLocation() {
        visionPortal.resumeStreaming();
        return pdp.getPosition();
    }

    public int getPixelHorizontalOffset() {
        return pixelPipeline.getCenterOffset();
    }

    /**
     * GROUP 2
     * @return true if the robot is in front of a piece of tape approximately perpendicular to the camera view, false otherwise
     */
    public boolean isRobotBeforeTape(boolean isRedTeam) {
        return tdp.isBeforeTape(isRedTeam);
    }

    public void moveToPixel() {
        double ERROR_THRESHOLD = 50;

        int pixelOffset = getPixelHorizontalOffset();
        while (Math.abs(pixelOffset) > ERROR_THRESHOLD) {
            if (pixelOffset < 0)
                drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            else
                drive.driveFieldCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            pixelOffset = getPixelHorizontalOffset();
        }
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
                    drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
                    break;
                case 2:
                    // right location
                    drive.driveRobotCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
                    break;
                default:
                    // center location
                    drive.driveRobotCentric(0, 1 * SpeedCoefficients.getForwardSpeed(), 0);
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
            drive.driveFieldCentric(0, 0, rotOff * 1 * SpeedCoefficients.getTurnSpeed()); // times some scaling factor (temporarily at 1)
        }
    }
}
