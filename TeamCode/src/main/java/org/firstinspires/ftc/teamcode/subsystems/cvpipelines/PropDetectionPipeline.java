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
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PropDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    boolean isRedTeam = true;

    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar blueUpperRange = new Scalar(100,157,255);  // Range needs fixing
    public static Scalar blueLowerRange = new Scalar(62,66,41);

    public static Scalar redUpperRange1 = new Scalar(5,255,160);  // Range needs fixing
    public static Scalar redLowerRange1 = new Scalar(0,34,106);

    public static Scalar redUpperRange2 = new Scalar(255, 124, 255);  // Range needs fixing
    public static Scalar redLowerRange2 = new Scalar(168, 62, 126);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public Point centerPoint;

    public int place;

    public int x;

    public int y;


    public PropDetectionPipeline(Telemetry telemetry, boolean isRedTeam){
        this.isRedTeam = isRedTeam;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        this.telemetry = telemetry;
        hasStarted.set(true);
        height = input.height();
        width = input.width();
        
        this.sub = input;
        
        double maxTotalArea = Double.MIN_VALUE;
        int best_idx = 0;
        
        for (int i=0; i<3; i++){
            Rect crop = new Rect(width*i/3,0,width/3, height);
            Imgproc.rectangle(input, crop, new Scalar(255, 255, 0));
            this.sub = new Mat(input, crop);
            double totalArea = getTotalContourArea();
            if (totalArea>maxTotalArea){
                maxTotalArea = totalArea;
                best_idx = i;
            }
            
        telemetry.addData("best idx", best_idx);
        telemetry.update();
        return input;
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
        double totalArea = 0;
        for (int idx = 0; idx<contourList.size(); idx++) {
            double area = Imgproc.contourArea(contourList.get(idx));
            totalArea += area;
        }

        return totalArea;
    }
}
