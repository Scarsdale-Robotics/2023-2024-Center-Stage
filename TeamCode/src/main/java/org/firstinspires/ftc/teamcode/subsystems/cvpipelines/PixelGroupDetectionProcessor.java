package org.firstinspires.ftc.teamcode.subsystems.cvpipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class PixelGroupDetectionProcessor implements VisionProcessor {
    enum PixelColor {
        WHITE(new Scalar(0, 0, 178.5), new Scalar(255, 26.9, 255), true),
        YELLOW(new Scalar(18.4, 111.9, 168.6), new Scalar(36.8, 225.0, 255.0), true),
        PURPLE(new Scalar(124.7, 36.8, 58.1), new Scalar(151.6, 117.6, 242.3), true),
        GREEN(new Scalar(38.3, 66.6, 83.6), new Scalar(55.3, 174.3, 255.0), true);
        public final Scalar UPPER;
        public final Scalar LOWER;
        public final boolean display;
        PixelColor(Scalar lower, Scalar upper, boolean display) {
            this.UPPER = upper; this.LOWER = lower; this.display = display;
        }
    }

    private static final PixelColor[] colors = new PixelColor[]{PixelColor.WHITE, PixelColor.YELLOW, PixelColor.PURPLE, PixelColor.GREEN};

    private Mat frame;
    public int getCameraWidth() {
        return frame.width();
    }

    private int centX = 0;
    private int centY = 0;

    public Point getPixelsCenter() {
        return new Point(centX, centY);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        frame = input;
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat inRange = new Mat();
        Mat morph = new Mat();
        Mat union = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(15, 15));
        for (PixelColor c : colors) {
            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);
            Imgproc.dilate(inRange, morph, kernel);
            Imgproc.erode(morph, morph, kernel);
            if (c == PixelColor.WHITE) {
                union = morph.clone();
            }
            Core.bitwise_or(morph, union, union);
        }
        long xSum = 0;
        long ySum = 0;
        int detections = 0;
        for (int x = 0; x < union.cols(); x++) {
            for (int y = 0; y < union.rows(); y++) {
                if (union.get(y, x)[0] != 0) {
                    xSum += x;
                    ySum += y;

                    detections++;
                }
            }
        }
        centX = (int) (xSum / Math.max(detections, 1));
        centY = (int) (ySum / Math.max(detections, 1));
//        Imgproc.circle(input, new Point(centX, centY), 8, new Scalar(0, 0, 255), 4);
//        Imgproc.circle(input, new Point(centX, centY), 0, new Scalar(0, 0, 255), 5);\
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint p = new Paint();
        p.setColor(Color.BLUE);
        p.setStrokeWidth(4);
        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }
}
