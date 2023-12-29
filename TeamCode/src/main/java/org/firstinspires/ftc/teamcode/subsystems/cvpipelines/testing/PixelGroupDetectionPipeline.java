package org.firstinspires.ftc.teamcode.subsystems.cvpipelines.testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class PixelGroupDetectionPipeline extends OpenCvPipeline {
    // CONVERSION FACTORS:
    // 179, 255, 255
    // NORMAL H * 0.5
    // NORMAL S * 2.55
    // NORMAL V * 2.55
    enum Color {
        WHITE(new Scalar(179, 10, 255), new Scalar(0, 0, 111), true),
        YELLOW(new Scalar(40, 255, 255), new Scalar(28, 166, 166), true),
        PURPLE(new Scalar(154, 255, 255), new Scalar(136, 166, 166), true),
        GREEN(new Scalar(70, 255, 255), new Scalar(54, 166, 166), true);
        public final Scalar UPPER;
        public final Scalar LOWER;
        public final boolean display;
        Color(Scalar upper, Scalar lower, boolean display) {
            this.UPPER = upper; this.LOWER = lower; this.display = display;
        }
    }
    private static final Color[] colors = new Color[]{Color.WHITE, Color.YELLOW, Color.PURPLE, Color.GREEN};
    @Override
    public Mat processFrame(Mat input) {
        Mat output = input;
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3, 3));

        ArrayList<Pixel> pixels = new ArrayList<>();
        for (Color c : colors) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat inRange = new Mat();
            Mat denoisedInRange = new Mat();
            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);

            Imgproc.morphologyEx(inRange, denoisedInRange, Imgproc.MORPH_DILATE, kernel);
            Imgproc.findContours(denoisedInRange, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            if (c == Color.WHITE)
            {
                Imgproc.cvtColor(denoisedInRange, output, Imgproc.COLOR_GRAY2RGB);
            }
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint cont = contours.get(i);
                MatOfPoint2f c2f = new MatOfPoint2f(cont.toArray());
                double peri = Imgproc.arcLength(c2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(c2f, approx, 0.05 * peri, true);

                Point[] points = approx.toArray();
                if (points.length == 6 && Imgproc.contourArea(cont) > input.width() / 10.0 && c.display) {
                    pixels.add(new Pixel(cont, c));
                    Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(input, c.name(), new Point(contours.get(i).get(0, 0)), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                }
            }

        }
        return output;
    }
    public static class Pixel {
        public MatOfPoint contour;
        public Color color;
        public Pixel(MatOfPoint contour, Color color) {
            this.contour = contour;
            this.color = color;
        }
    }
}
