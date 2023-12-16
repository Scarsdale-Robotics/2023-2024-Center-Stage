package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelDetectionPipeline;

import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.WhitePixelDetectionPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CVSubsystem extends SubsystemBase {
    private OpenCvCamera camera;
    private DriveSubsystem drive;

    private final int SAMPLE_COUNT = 200;
    private final long SAMPLE_WAIT_MILLISECONDS = 25;
    private final int LOCATION_LEFT   =  0;
    private final int LOCATION_CENTER =  1;
    private final int LOCATION_RIGHT  =  2;
    private final int NO_LOCATION     = -1;
    private Telemetry telemetry;
    final private ElapsedTime runtime = new ElapsedTime();

    private final double NO_ROTATIONAL_OFFSET = -50000.0;
    private final double NO_DISTANCE = -50000.0;
    private final double ERROR   =  3.0;
    private final double ERROR_ALIGNMENT = 2;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final WebcamName cameraName;
    private PropDetectionPipeline propProcessor;
    private PixelDetectionPipeline pixelProcessor;
    private WhitePixelDetectionPipeline whitePixelProcessor;
    private boolean isRedTeam;
    private LinearOpMode opMode;
    public CVSubsystem(OpenCvCamera camera, WebcamName cameraName, DriveSubsystem drive, Telemetry telemetry, boolean isRedTeam, LinearOpMode opMode) {
//        tdp = new TapeDetectionPipeline();
//        camera.setPipeline(tdp);
        this.camera = camera;
        this.drive = drive;
        this.telemetry = telemetry;
        this.cameraName = cameraName;
        this.isRedTeam = isRedTeam;
        this.opMode = opMode;
        runtime.reset();
        // create AprilTagProcessor and VisionPortal
        initAprilTag();


//        pixelPipeline = new PixelDetectionPipeline();
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
        // Create the AprilTag processor.
        propProcessor = new PropDetectionPipeline(isRedTeam);
        pixelProcessor = new PixelDetectionPipeline();
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

        builder.setCamera(cameraName);
        builder.setAutoStopLiveView(false); // keep camera on when not processing

        builder.setCameraResolution(new Size(640, 480)); // android.util

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(pixelProcessor);
        builder.addProcessor(propProcessor);
//        builder.addProcessors(aprilTag, pixelProcessor, propProcessor);

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
//        visionPortal.resumeStreaming();
        //this is old for testing apriltag team prop thingy

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
            return -2;
        }

        return NO_LOCATION;
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return a double representing the amount the robot should turn to be "parallel" to the AprilTag
     */
    public double getAprilTagRotationalOffset(int tagID) { // return yaw
//        visionPortal.resumeStreaming();

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

    public double getAprilTagDistance(Integer... tagIDs) {
//        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = Double.MAX_VALUE;
        Set<Integer> tagSearchSet = new HashSet<>(Arrays.asList(tagIDs));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && tagSearchSet.contains(detection.id)) {
                double d = detection.ftcPose.range;
                if (d < tagDistance)
                    tagDistance = d;
            }
        }
//        visionPortal.stopStreaming();

        return tagDistance;
    }

    public double getAprilTagDistance(int tagID) {
//        visionPortal.resumeStreaming();

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
//        visionPortal.stopStreaming();

        return tagDistance;
    }

    /**
     * GROUP 2
     * @return whether the AprilTag is left, center, or right in the camera view based on a mode of samples
     */
    public int getPropLocation() throws InterruptedException {
        int propLocation = -1;
        ArrayList<Integer> samples = new ArrayList<>();
        sleepFor(1000);
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            if (!opMode.opModeIsActive()) break;
            propLocation = propProcessor.getPosition();
            if (-1 < propLocation && propLocation < 3) samples.add(propLocation);
            sleepFor(SAMPLE_WAIT_MILLISECONDS);
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
        return propLocation;
    }

    public int getPixelHorizontalOffset() {
//        visionPortal.resumeStreaming();
        return pixelProcessor.getCenterOffset();
    }

    public int getWhitePixelHorizontalOffset() {
        return whitePixelProcessor.getCenterOffset();
    }

    public int getCameraWidth() {
        return whitePixelProcessor.getCameraWidth();
    }

    public double getAprilTagHorizontalOffset(int tagID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = NO_DISTANCE;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    tagDistance = detection.ftcPose.z;
                    return tagDistance;
                }
            }
        }
        return tagDistance;
    }

    /**
     * GROUP 2
     * @return true if the robot is in front of a piece of tape approximately perpendicular to the camera view, false otherwise
     */
//    public boolean isRobotBeforeTape(boolean isRedTeam) {
//        visionPortal.resumeStreaming();
//        return tdp.isBeforeTape(isRedTeam);
//    }

    public void moveToPixel() {
        double ERROR_THRESHOLD = 50;

        int pixelOffset = getPixelHorizontalOffset();
        while (Math.abs(pixelOffset) > ERROR_THRESHOLD) {
            if (pixelOffset < 0){
                drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }else{
                drive.driveFieldCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }
            pixelOffset = getPixelHorizontalOffset();
        }
    }

    public void moveToWhitePixel() {
        double ERROR_THRESHOLD = 50;
        int width = getCameraWidth();
        int pixelOffset = getWhitePixelHorizontalOffset();
        while (Math.abs(pixelOffset) > ERROR_THRESHOLD) {
            if (pixelOffset < 0){ //NOT TESTED IDEA, PROB NEEDS FIXING: GO FASTER WHEN DISTANCE TO PIXEL STACK IS LARGE, THEN SLOW DOWN AS IT NEARS IT.
                drive.driveRobotCentric((Math.abs(pixelOffset)*2/width) * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }else{
                drive.driveFieldCentric(-(Math.abs(pixelOffset)*2/width) * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }
            pixelOffset = getWhitePixelHorizontalOffset();
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



        while (getAprilTagHorizontalOffset(tagID) > DISTANCE_THRESHOLD) {
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
            drive.driveByEncoder(0, 0, 0,0); //brake
        }
    }

    /**
     * aligns such that a ray representing a robot's camera direction is (anti-)parallel to a ray extending "outwards" from the center of a given AprilTag
     * @param tagID the id of the AprilTag from the 36h11 family to align with
     */
    public void alignParallelWithAprilTag(int tagID) {
        double rotOff = getAprilTagRotationalOffset(tagID); //counter clockwise yaw is positive
        telemetry.addData("err", Math.abs(rotOff));
        if (rotOff == NO_ROTATIONAL_OFFSET) {
            // maybe do something
        } else {
            while (Math.abs(getAprilTagRotationalOffset(tagID)) > ERROR_ALIGNMENT && opMode.opModeIsActive()) {
                //assume that we don't need to optimize getAprilTagRotationalOffset(tagID) since it runs anyway
                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed()); // times some scaling factor (temporarily at 1)
            }
            drive.driveByEncoder(0, 0, 0, 0);
//            while (Math.abs(rotOff) > ERROR_ALIGNMENT && opMode.opModeIsActive()) {
//                drive.driveByEncoder(0, 0, 0.2, (double) 1770 * rotOff / 180);
//            }
//            drive.driveByEncoder(0, 0, 0, 0);
        }

    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }
}
