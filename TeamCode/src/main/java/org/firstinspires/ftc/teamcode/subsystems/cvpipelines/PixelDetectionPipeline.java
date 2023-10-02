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

public class PixelDetectionPipeline extends OpenCvPipeline {
    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar upperRange = new Scalar(34, 255, 255);  // hsv scale: 179, 255, 255
    //white: (179, 25, 255)
    //green: (70, 255, 255)
    //yellow: (34, 255, 255)
    public static Scalar lowerRange = new Scalar(13, 155, 53);
    //white: (0, 0, 138)
    //green: (40, 155, 153)
    //yellow: (13,155, 28)

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public int contour_dim_ratio;

    public Mat processFrame(Mat input) {
        hasStarted.set(true);
        //open cv defaults to bgr, but we are completely in rgb/hsv, this is because pipeline's input/output rgb
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
//            contour_dim_ratio=b.width/b.height;
//            if (contour_dim_ratio < 1) {
            pixelContour.add(contour);

            Imgproc.drawContours(sub, pixelContour, 0, new Scalar(0, 255, 255), 2);
            posX = b.x + b.width / 2;
            posY = b.y + b.height / 2;
            Imgproc.circle(sub, new Point(posX, posY), 2, new Scalar(255, 0, 255)); //rgb DRAW CIRCLE
//            }

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
