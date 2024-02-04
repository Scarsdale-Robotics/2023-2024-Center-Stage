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
//    public static Scalar blueLowerRange = new Scalar(86,41,111);
    public static Scalar blueLowerRange = new Scalar(86,33,88);

    public static Scalar redUpperRange1 = new Scalar(66,255,255);  // Range needs fixing
    public static Scalar redLowerRange1 = new Scalar(0,95,196);

    public static Scalar redUpperRange2 = new Scalar(63, 255, 199);  // Range needs fixing
    public static Scalar redLowerRange2 = new Scalar(0, 112, 73);

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


    public double getTotalContourArea() {
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);

        List<MatOfPoint> contourList = new ArrayList<>();
        if (isRedTeam){
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsvmat, redLowerRange1, redUpperRange1, mask1);
            Core.inRange(hsvmat, redLowerRange2, redUpperRange2, mask2);
            Imgproc.findContours(mask1, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(mask2, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }else {
            Mat mask = new Mat();
            Core.inRange(hsvmat, blueLowerRange, blueUpperRange, mask);
            Imgproc.findContours(mask, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }
        for (int i = 0; i < contourList.size(); i++)
            Imgproc.drawContours(output, contourList, i, new Scalar(0, 0, 255));
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
        this.output = input;

        double maxTotalArea = Double.MIN_VALUE;
        int best_idx = -999;

        for (int i=0; i<3; i++) {
            Rect crop = new Rect(width*i/3,height * 17 / 24,width/3, height * 7 / 24);
            Imgproc.rectangle(input, crop, new Scalar(255, 255, 0));
            this.sub = new Mat(input, crop);
            double totalArea = getTotalContourArea();
            if (totalArea>maxTotalArea){
                maxTotalArea = totalArea;
                best_idx = i;
            }
        }
        position = best_idx;

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}