package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FilterPipeline extends OpenCvPipeline {
    private int sleeveSide;

    Mat temp;
    public int answer;

    @Override
    public Mat processFrame(Mat input) {
        int midY = input.rows()/2;
        int midX = input.cols()/2;
        double[] centerPoint = input.get(midY, midX);
        int ix = findGreatestLocation(centerPoint);
        answer = ix;

        return input;
    }

    public int getAnswer() {
        return answer;
    }

    public int findGreatestLocation(double[] j) {
        int biggestIndex = 0;
        for (int i = 0; i < j.length; i++)
        {
            if (j[i] > j[biggestIndex]){
                biggestIndex = i;
            }
        }
        return biggestIndex;
    }

    public contourData findLargestContour(Mat input, Scalar lower, Scalar upper) {
        Mat rangeMat = new Mat();
        Imgproc.cvtColor(input, rangeMat, Imgproc.COLOR_BGR2HSV);

        Core.inRange(rangeMat, lower, upper, rangeMat);

        List<MatOfPoint> contourList = new ArrayList<>();

        Imgproc.findContours(rangeMat, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = Double.MIN_VALUE;
        MatOfPoint maxContour = null;
        for (MatOfPoint m : contourList) {
            double area = Imgproc.contourArea(m);
            if (area > maxArea) {
                maxArea = area;
                maxContour = m;
            }
        }
        return new contourData(maxArea, maxContour);
    }

    public class contourData {
        public double area;
        public MatOfPoint contour;

        public contourData(double _area, MatOfPoint _contour) {
            area = _area;
            contour = _contour;
        }
    }
}