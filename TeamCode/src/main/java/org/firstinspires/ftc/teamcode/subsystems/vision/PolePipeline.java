package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class PolePipeline extends OpenCvPipeline {
    Mat temp = new Mat();
    Mat temp2 = new Mat();
    Scalar lowerYellow = new Scalar(80, 50, 20);
    Scalar upperYellow = new Scalar(100, 360, 360);

    Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    AtomicInteger location = new AtomicInteger(-1);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_BGR2HSV);
        Core.inRange(temp, lowerYellow, upperYellow, temp);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_OPEN, morphKernel);

        List<MatOfPoint> contourList = new ArrayList<>();

        Imgproc.findContours(temp, contourList, temp2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = Integer.MIN_VALUE;
        MatOfPoint maxContour = null;

        for (MatOfPoint m : contourList) {
            double area = Imgproc.contourArea(m);
            if (area > maxArea) {
                maxArea = area;
                maxContour = m;
            }
        }

        Imgproc.drawContours(input, contourList, -1, new Scalar(255, 0, 0));

        if(maxContour != null) {
            Rect bound = Imgproc.boundingRect(maxContour);
            location.set(bound.x);
        } else {
            location.set(-1);
        }

        return input;
    }

    public int getXPosition() {
        return location.get();
    }
}
