package org.firstinspires.ftc.teamcode.subsystems.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ExPipeline extends OpenCvPipeline {

    /*
       OpenCV HSV Ranges:           H (0-179), S (0-255), V (0-255)
       "Standard" HSV Ranges:       H (0-360), S (0-100), V (0-100)
       Standard to OCV conversion:  x0.4972  , x2.550   , x2.550
       [multiply the standard HSV value by the indicated value to get the OCV HSV value]
    */
    public final Scalar HSV_UPPER_BLUE = new Scalar(134, 255, 255);
    public final Scalar HSV_LOWER_BLUE = new Scalar(84, 100, 69);

    public final Scalar RGB_CONTOUR_OUTLINE = new Scalar(0, 169, 0);

    private Mat rgbOutput;
    private Mat frame;

    @Override
    public Mat processFrame(Mat input) {
        frame = input;
        rgbOutput = input;
        List<MatOfPoint> contourList = getContourList(input, HSV_LOWER_BLUE, HSV_UPPER_BLUE);
        for (int i = 0; i < contourList.size(); ++i) {
            Imgproc.drawContours(rgbOutput, contourList, i, RGB_CONTOUR_OUTLINE, 1);
        }
        return rgbOutput;
    }

    public boolean isEnoughBlue(double areaThreshold) {
        for (MatOfPoint contour : getContourList(frame, HSV_LOWER_BLUE, HSV_UPPER_BLUE)) {
            if (Imgproc.contourArea(contour) >= areaThreshold) {
                return true;
            }
        }
        return false;
    }

    private List<MatOfPoint> getContourList(Mat src, Scalar lowerRange, Scalar upperRange) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(src, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
        Core.inRange(hsvFrame, lowerRange, upperRange, inRange);

        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(inRange, contourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contourList;
    }
}