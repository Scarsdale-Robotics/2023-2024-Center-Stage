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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

// 320 x 240 camera resolution standard

public class PixelDetectionPipeline extends OpenCvPipeline {
    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static final Scalar upperWhite = new Scalar(80, 50, 255);
    public static final Scalar upperGreen = new Scalar(80, 255, 255);
    public static final Scalar upperYellow = new Scalar(34, 255, 255);
    public static final Scalar upperPurple = new Scalar(160, 180, 200);

    public static final Scalar lowerWhite = new Scalar(0, 30, 170);
    public static final Scalar lowerGreen = new Scalar(40, 90, 110);
    public static final Scalar lowerYellow = new Scalar(13, 135, 28);
    public static final Scalar lowerPurple = new Scalar(140, 25, 95);

    public static Scalar upperRange = upperWhite;
    public static Scalar lowerRange = lowerWhite;

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public int contourDimRatio;

    public Mat processFrame(Mat input) {
        hasStarted.set(true);

        // Open cv defaults to BGR, but we are completely in RGB/HSV, this is because pipeline's input/output RGB
        height = input.height()/3;
        width = input.width();
        int y = 2 * height;
        int x = 0;
        this.sub = input;
        this.frame = input;
        int posY = height / 2;
        int posX = width / 2;

        MatOfPoint contour = getPixelContour();

        if(contour != null) {
            List<MatOfPoint> pixelContour = new ArrayList<>();
            Rect b = Imgproc.boundingRect(contour);

            // contourDimRatio=b.width/b.height;
            // if (contourDimRatio < 1) {

            pixelContour.add(contour);

            Imgproc.drawContours(sub, pixelContour, 0, new Scalar(0, 255, 255), 2);
            posX = b.x + b.width / 2;
            posY = b.y + b.height / 2;
            Imgproc.circle(sub, new Point(posX, posY), 2, new Scalar(255, 0, 255)); // RGB DRAW CIRCLE

            // }

        }

        int offset = posX - (width / 2 + 40);
        lateralOffset.set(offset);
        return input;
    }

    public MatOfPoint getPixelContour() {
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
        Core.inRange(hsvmat, lowerRange, upperRange, inRange);

        List<MatOfPoint> whiteContourList = new ArrayList<>();

        Imgproc.findContours(inRange, whiteContourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < whiteContourList.size(); i++) {
            Imgproc.drawContours(sub, whiteContourList, i, new Scalar(255, 255, 0), 2);
        }

        double maxArea = Double.MIN_VALUE;
        MatOfPoint maxContour = null;
        for (MatOfPoint m : whiteContourList) {
            double area = Imgproc.contourArea(m);

            if (area > maxArea) {
                maxArea = area;
                maxContour = m;
            }
        }

        return maxContour;
    }
}
