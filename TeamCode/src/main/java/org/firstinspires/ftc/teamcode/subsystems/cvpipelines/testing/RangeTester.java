package org.firstinspires.ftc.teamcode.subsystems.cvpipelines.testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RangeTester extends OpenCvPipeline {
    public static Scalar LOWER_RANGE = new Scalar(0, 0, 0);
    public static Scalar UPPER_RANGE = new Scalar(255, 100, 100);
    public static int MORPH_MODE = 3;
    public static int MORPH_SIZE = 15;

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat inRange = new Mat();
        Core.inRange(hsv, LOWER_RANGE, UPPER_RANGE, inRange);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(MORPH_SIZE, MORPH_SIZE));
        Mat morph = new Mat();
        Imgproc.morphologyEx(inRange, morph, MORPH_MODE, kernel);
        Mat out = new Mat();
        Imgproc.cvtColor(morph, out, Imgproc.COLOR_GRAY2RGB);

        return out;
    }
}
