package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTagDetectionPipeline;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

public class SleeveDetector {
    public AprilTagDetectionPipeline ap;
    OpenCvCamera phoneCam;
    private HashMap<Integer, Integer> idToParkingSpace = new HashMap<>();

    public SleeveDetector(OpenCvCamera camera, Telemetry telemetry) {
        phoneCam = camera;

        ap = new AprilTagDetectionPipeline(telemetry);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        phoneCam.setPipeline(ap);

        idToParkingSpace.put(15, 0);
        idToParkingSpace.put(9, 1);
        idToParkingSpace.put(16, 2);

        ap.setDecimation(1f);
    }

    public int getParkingLocation() {
        int largestedgelen = Integer.MIN_VALUE;
        int answer = -1;

        ArrayList<AprilTagDetection> detections = ap.getLatestDetections();

        for (AprilTagDetection detection : detections) {
            for (int ix = 0; ix < 4; ix++) {
                int jx = (ix + 1) % 4;

                Point ip = detection.corners[ix];
                Point jp = detection.corners[jx];

                int edgelen = (int) Math.round(Math.sqrt(Math.pow(Math.abs(ip.x - jp.x), 2) + Math.pow(Math.abs(ip.y - jp.y), 2)));
                if (edgelen > largestedgelen) {
                    largestedgelen = edgelen;

                    int detectedid = detection.id;
                    answer = idToParkingSpace.getOrDefault(detectedid, -1);
                }
            }
        }
        return answer;
    }

    public void close() {
        phoneCam.closeCameraDevice();
    }

}
