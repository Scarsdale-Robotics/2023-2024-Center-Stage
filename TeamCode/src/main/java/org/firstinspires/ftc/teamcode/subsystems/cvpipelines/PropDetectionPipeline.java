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

public class PropDetectionPipeline extends OpenCvPipeline {
    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar upperRange = new Scalar(110.5,255,255);  // Range needs fixing
    public static Scalar lowerRange = new Scalar(86,214,64);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public int contour_dim_ratio;

    // ADA's idea: COMPARE RATIOS OF WIDTH : HEIGHT TO BE MORE ACCURATE ON DETECTING TAPE AND POLE
    @Override
    public Mat processFrame(Mat input) {
        hasStarted.set(true);
        //open cv defaults to bgr, but we are completely in rgb/hsv, this is because pipeline's input/output rgb
        height = input.height()/3;
        width = input.width();
        int y = 2 * height;
        int x = 0;
        this.sub = input;
        this.frame = input;
        int posy = height/2;
        int posx = width / 2;
        MatOfPoint contour = getConeContour();
        if(contour != null) {
            List<MatOfPoint> coneContour = new ArrayList<>();
            Rect b = Imgproc.boundingRect(contour);
//            contour_dim_ratio=b.width/b.height;
//            if (contour_dim_ratio < 1) {
            coneContour.add(contour);
            Imgproc.drawContours(sub, coneContour, 0, new Scalar(255, 255, 0), 2);
            posx = b.x + b.width / 2;
            posy = b.y + b.height / 2;
            Imgproc.circle(sub, new Point(posx, posy), 2, new Scalar(255, 255, 0)); //rgb DRAW CIRCLE
//            }

        }

        int offset = posx - (width / 2 + 40);
        lateralOffset.set(offset);

        return input;
    }

    public MatOfPoint getConeContour() {
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
        Core.inRange(hsvmat, lowerRange, upperRange, inRange);

        List<MatOfPoint> blueContourList = new ArrayList<>();
        Imgproc.findContours(inRange, blueContourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = Double.MIN_VALUE;
        MatOfPoint maxContour = null;
        for (MatOfPoint m : blueContourList) {
            double area = Imgproc.contourArea(m);
            if (area > maxArea) {
                maxArea = area;
                maxContour = m;
            }
        }

        return maxContour;
    }
}