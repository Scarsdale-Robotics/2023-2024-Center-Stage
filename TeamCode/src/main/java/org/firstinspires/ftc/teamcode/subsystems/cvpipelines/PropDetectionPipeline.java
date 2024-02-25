package org.firstinspires.ftc.teamcode.subsystems.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PropDetectionPipeline implements VisionProcessor {
    boolean isRedTeam;

    public Mat sub;
    private Mat output;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar blueUpperRange = new Scalar(134,255,255);  // Range needs fixing
    public static Scalar blueLowerRange = new Scalar(86,41,111);
//    public static Scalar blueLowerRange = new Scalar(78,33,88);

    public static Scalar redUpperRange1 = new Scalar(66,255,255);  // Range needs fixing
    public static Scalar redLowerRange1 = new Scalar(0,41,69);

    public static Scalar redUpperRange2 = new Scalar(179, 255, 255);  // Range needs fixing
    public static Scalar redLowerRange2 = new Scalar(155, 41, 69);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);

    public int position = -999;
    // chicken nugget nat nuo tao
    // kocmoc
    // co|-o3 HepyWNmhN Pec

    public PropDetectionPipeline(boolean isRedTeam){
        this.isRedTeam = isRedTeam;
    }

    public int getPosition(){
        return position;
    }

    public final boolean DRAW_OUTPUT = true;  // set off for comp
    public double getTotalContourArea() {
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(20, 20));
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);

        List<MatOfPoint> contourList = new ArrayList<>();
        Mat mask = new Mat();
        if (isRedTeam){
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsvmat, redLowerRange1, redUpperRange1, mask1);
            Core.inRange(hsvmat, redLowerRange2, redUpperRange2, mask2);
            Imgproc.morphologyEx(mask1, mask1, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask2, mask2, Imgproc.MORPH_OPEN, kernel);
            Core.bitwise_or(mask1, mask2, mask);
            Imgproc.findContours(mask, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        } else {
            Core.inRange(hsvmat, blueLowerRange, blueUpperRange, mask);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.findContours(mask, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }
        if (DRAW_OUTPUT) {
            Mat black = new Mat(sub.size(), sub.type(), new Scalar(0, 69, 0));
            sub.copyTo(black, mask);
            sub = black;
        }
//        for (int i = 0; i < contourList.size(); i++)
//            Imgproc.drawContours(output, cont ourList, i, new Scalar(0, 0, 255));
        double totalArea = 0;
        for (int idx = 0; idx<contourList.size(); idx++) {
            double area = Imgproc.contourArea(contourList.get(idx));
            totalArea += area;
        }

        return totalArea;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        height = input.height();
        width = input.width();
        hasStarted = new AtomicBoolean(true);

//        Core.transpose(input, input);
//        Core.flip(input, input, 0);  // Switch flipCode to 0 if inverted
        this.sub = input;
//        this.output = input;
        this.output = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));

        double maxTotalArea = Double.MIN_VALUE;
        int best_idx = -999;

        for (int i=0; i<3; i++) {
            Rect crop = new Rect(width*i/3,height * 17 / 24,width/3, height * 7 / 24);
            this.sub = new Mat(input, crop);
            double totalArea = getTotalContourArea();
            if (totalArea>maxTotalArea){
                maxTotalArea = totalArea;
                best_idx = i;
            }
            String posName;
            if (i == 0) posName = "LEFT";
            else if (i == 1) posName = "CENTER";
            else posName = "RIGHT";
            if (DRAW_OUTPUT) {
                Imgproc.putText(sub, posName, new Point(10, 36), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 3);
                sub.copyTo(output.submat(crop));
                Imgproc.rectangle(output, crop, new Scalar(255, 255, 0));
            }
        }
        position = best_idx;
        if (DRAW_OUTPUT) {
            String posName = "NO PROP DETECTED";
            if (position == 0) posName = "LEFT";
            else if (position == 1) posName = "CENTER";
            else if (position == 2) posName = "RIGHT";
            Imgproc.putText(output, posName, new Point(10, 50), Imgproc.FONT_HERSHEY_PLAIN, 3, isRedTeam ? new Scalar (255, 0, 0) : new Scalar(0, 0, 255), 6);
            Rect crop = new Rect(width*position/3,height * 17 / 24,width/3, height * 7 / 24);
            Imgproc.rectangle(output, crop, new Scalar(0, 255, 0), 10);
        }
        output.copyTo(input);
        return output;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}