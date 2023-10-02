package org.firstinspires.ftc.teamcode.subsystems.cvpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input,new Rect(0, 130, 318, 50), new Scalar(255, 0, 0), 2);
        Rect crop = new Rect(0, 130, 318, 50);
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat inRange = new Mat();
        Core.inRange(hsvMat, new Scalar(100, 90, 80), new Scalar(125, 255, 255), inRange);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(inRange, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0), 1);
        }
        return input;

    }
}
