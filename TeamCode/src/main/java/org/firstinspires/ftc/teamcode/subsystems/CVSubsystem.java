package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelGroupDetectionProcessor;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelDetectionPipeline;

import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.WhitePixelDetectionPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CVSubsystem extends SubsystemBase {
    private OpenCvCamera camera;
    private DriveSubsystem drive;

    public final int SAMPLE_COUNT = 200;
    public final long SAMPLE_WAIT_MILLISECONDS = 25;
    public final int LOCATION_LEFT   =  0;
    public final int LOCATION_CENTER =  1;
    public final int LOCATION_RIGHT  =  2;
    public final int NO_LOCATION     = -1;
    private Telemetry telemetry;
    final private ElapsedTime runtime = new ElapsedTime();

    public final double NO_ROTATIONAL_OFFSET = -50000.0;
    public final double NO_DISTANCE = -50000.0;
    public final double ERROR   =  3.0;
    public final double ERROR_ALIGNMENT = 4;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final WebcamName cameraName;
    private PropDetectionPipeline propProcessor;
    private PixelDetectionPipeline pixelProcessor;
    private WhitePixelDetectionPipeline whitePixelProcessor;
    private PixelGroupDetectionProcessor pixelGroupProcessor;
    private boolean isRedTeam;
    private LinearOpMode opMode;
    private static final double[][] APRIL_TAG_LOCATIONS = new double[][]{
            {1.25, 0.5}, {1.5, 0.5}, {1.75, 0.5},
            {4.25, 0.5}, {4.5, 0.5}, {4.75, 0.5},
            {4.75, 6}  , {4.5, 6}  ,
            {1.25, 6}  , {1.5, 6}
    };
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
        whitePixelProcessor = new WhitePixelDetectionPipeline();
        pixelGroupProcessor = new PixelGroupDetectionProcessor();
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
        builder.addProcessors(propProcessor, pixelGroupProcessor);
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
    public int getPropLocation() {
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

    public double getWhitePixelDiameterPx() {
        return 0; // TODO: WRITE
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

    private double[] lastKnownPos = POSITION_UNKNOWN;
    public static final double[] POSITION_UNKNOWN = new double[]{-1, -1, -1};
    /**
     * returns the x,y,rotation position of the center of the robot based on apriltag positions in tiles. blue-backdrop corner is considered 0,0.
     * facing backdrop is zero rotation, right is more rotation
     * offset params are positive towards the direction the camera is facing. the camera is assumed to be facing either "robot-forward" or "robot-backward"
     */
    public double[] getPosition(double cameraCenterOffsetX, double cameraCenterOffsetY) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() == 0) return lastKnownPos;
        ArrayList<Double> xEst   = new ArrayList<>();
        ArrayList<Double> yEst   = new ArrayList<>();
        ArrayList<Double> rotEst = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            double rotOff = (detection.ftcPose.x > 0 ? 1 : -1) * detection.ftcPose.yaw;
            // 24 inches per tile
            double xOff   = (Math.cos(detection.ftcPose.bearing - detection.ftcPose.yaw) * detection.ftcPose.range) / 24 - cameraCenterOffsetX;
            double yOff   = (Math.sin(detection.ftcPose.bearing - detection.ftcPose.yaw) * detection.ftcPose.range) / 24 - cameraCenterOffsetY;
            if (detection.id < 7) {
                xEst.add(APRIL_TAG_LOCATIONS[detection.id][0] + yOff);
                yEst.add(APRIL_TAG_LOCATIONS[detection.id][1] + xOff);
                rotEst.add(rotOff);
            } else {
                xEst.add(APRIL_TAG_LOCATIONS[detection.id][0] - yOff);
                yEst.add(APRIL_TAG_LOCATIONS[detection.id][1] - xOff);
                rotEst.add(rotOff + 180);
            }
        }
        Collections.sort(xEst);
        Collections.sort(yEst);
        Collections.sort(rotEst);
        double xMed   = xEst.get(xEst.size() / 2);
        double yMed   = yEst.get(xEst.size() / 2);
        double rotMed = rotEst.get(xEst.size() / 2);
        lastKnownPos = new double[]{xMed, yMed, rotMed};
        return lastKnownPos;
    }

    public void moveToPixel() {
        double ERROR_THRESHOLD = 50;

        int pixelOffset = getPixelHorizontalOffset();
        while (Math.abs(pixelOffset) > ERROR_THRESHOLD) {
            if (pixelOffset < 0){
                drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }else{
                drive.driveRobotCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }
            pixelOffset = getPixelHorizontalOffset();
        }
    }

    public void moveToWhitePixel() {
        int width = getCameraWidth();
        double HORIZ_THRESHOLD = width / 11.1;
        double DIAM_THRESHOLD = width / 4.0;
        int pixelOffset = getWhitePixelHorizontalOffset();
        double pixelDiam = getWhitePixelDiameterPx();
        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDiam) < DIAM_THRESHOLD && opMode.opModeIsActive()) {
            drive.driveRobotCentric(Math.max(DIAM_THRESHOLD, pixelDiam) / DIAM_THRESHOLD, Math.min(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, 0);
            pixelOffset = getWhitePixelHorizontalOffset();
            pixelDiam = getWhitePixelDiameterPx();
        }
    }

    public Point getPixelsCenter() {

        return pixelGroupProcessor.getPixelsCenter();
    }
    public void moveToPixels() {
        int width = getCameraWidth();
        double HORIZ_THRESHOLD = width / 11.1;
        double DIST_THRESHOLD = width / 4.0;
        Point p = pixelGroupProcessor.getPixelsCenter();
        double pixelOffset = p.x;
        double pixelDist = p.y;
        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDist) < DIST_THRESHOLD && opMode.opModeIsActive()) {
            drive.driveRobotCentric(Math.max(DIST_THRESHOLD, pixelDist) / DIST_THRESHOLD, Math.min(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, 0);
            pixelOffset = getWhitePixelHorizontalOffset();
            pixelDist = getWhitePixelDiameterPx();
        }
    }

    public void moveToAprilTag(int tagID) {
        int width = getCameraWidth();
        double HORIZ_THRESHOLD = width / 11.1;
        double DIST_THRESHOLD = 22;
        double pixelOffset = getAprilTagHorizontalOffset(tagID);
        double pixelDist = getAprilTagDistance(tagID);
        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDist) > DIST_THRESHOLD) {
            drive.driveRobotCentric(Math.max(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, Math.max(DIST_THRESHOLD, pixelDist) / DIST_THRESHOLD, 0);
            pixelOffset = getWhitePixelHorizontalOffset();
            pixelDist = getWhitePixelDiameterPx();
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
            drive.driveByRectilinearEncoder(0, 0, 0);
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
