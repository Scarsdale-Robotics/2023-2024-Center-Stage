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
import java.util.Collections;
import java.util.List;

public class TestPipeline extends OpenCvPipeline {
    private Mat frame = new Mat();
    public int beforeTape = 0;
    public double largestArea_=0;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input,new Rect(0, 330, 640, 100), new Scalar(255, 0, 0), 2);
        Rect crop = new Rect(0, 330, 640, 100);
        Mat cropped = input.submat(crop);
        frame = cropped;
        beforeTape = isBeforeTape(false) ? 1 : 0;
        return frame;

    }
    public boolean isBeforeTape(boolean isRedTeam) {
        double areaThreshold = 50;
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat inRange = new Mat();
        double largestArea = 0;
        Core.inRange(hsvMat, new Scalar(100, 90, 80), new Scalar(125, 255, 255), inRange);
        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(inRange, contourList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contourList.size(); i++) {
            Imgproc.drawContours(frame, contourList, i, new Scalar(0, 255, 0), 1);
            MatOfPoint result = contourList.get(i); //all contour size list
            double area = Imgproc.contourArea(result);
            if (area > largestArea) {
                largestArea = area;
            }
            //get area of largest contour
            //set largestArea to that
        }
        largestArea_ = largestArea;
        if (largestArea > areaThreshold) {
            return true;
        } else {
            return false;
        }
    }
}
