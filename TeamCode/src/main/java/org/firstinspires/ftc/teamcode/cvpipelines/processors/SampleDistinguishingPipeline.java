package org.firstinspires.ftc.teamcode.cvpipelines.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleDistinguishingPipeline implements VisionProcessor {
    private Telemetry telemetry;
    public SampleDistinguishingPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    private ArrayList<Scalar> UPPER_COLOR_RANGES;
    private ArrayList<Scalar> LOWER_COLOR_RANGES;

    public void setColorRanges(boolean red, boolean blue, boolean yellow) {
        UPPER_COLOR_RANGES = new ArrayList<>();
        LOWER_COLOR_RANGES = new ArrayList<>();

        if (red) {
            UPPER_COLOR_RANGES.add(new Scalar(5, 255, 255));
            UPPER_COLOR_RANGES.add(new Scalar(180, 255, 255));
            LOWER_COLOR_RANGES.add(new Scalar(0, 50, 0));
            LOWER_COLOR_RANGES.add(new Scalar(175, 255, 255));
        }
        if (blue) {
            UPPER_COLOR_RANGES.add(new Scalar(100, 255, 255));
            LOWER_COLOR_RANGES.add(new Scalar(70, 50, 0));
        }
        if (yellow) {
            UPPER_COLOR_RANGES.add(new Scalar(74, 255, 255));
            LOWER_COLOR_RANGES.add(new Scalar(10, 50, 0));
        }
    }

    public static class Pose {
        double x,y,theta;
        public Pose(double x,double y,double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        telemetry.update();

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Mat inRange = null;

        for (int i=0;i<UPPER_COLOR_RANGES.size();i++) {
            Mat tr = new Mat();
            Core.inRange(hsv, UPPER_COLOR_RANGES.get(i), LOWER_COLOR_RANGES.get(i), tr);
            if (inRange == null) inRange = tr;
            else Core.bitwise_or(inRange, tr, inRange);
        }
        Core.inRange(hsv, YELLOW_LOWER_HSV, YELLOW_UPPER_HSV, inRange);


        Mat dilationKernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3, 3));
        Mat erosionKernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10));

        Imgproc.morphologyEx(inRange, inRange, Imgproc.MORPH_DILATE, dilationKernel);
        Imgproc.morphologyEx(inRange, inRange, Imgproc.MORPH_ERODE, erosionKernel);

//        Core.bitwise_not(frame, frame, inRange);
//        return frame;

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(inRange, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double mxca = Double.MAX_VALUE;
        MatOfPoint2f mxcact = null;

        for (int i=0;i<contours.size();i++) {
            MatOfPoint2f mOP2F = new MatOfPoint2f();
            contours.get(i).convertTo(mOP2F, CvType.CV_32FC2);
            double ca = Imgproc.contourArea(contours.get(i));
            if (ca < mxca) {
                mxcact = mOP2F;
                mxca = ca;
            }
        }

        telemetry.addData("hi", mxcact == null);
        if (mxcact == null) {
            yellowSamplePose = null;
            return null;
        }

        RotatedRect rect = Imgproc.minAreaRect(mxcact);

        Point[] vertices = new Point[4];
        rect.points(vertices);
        double maxLen = 0, maxLenAngle = 0;
        for (int i=0;i<4;i++) {
            Point p1 = vertices[i];
            Point p2 = vertices[(i+1)%4];
            double len = Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
            if (len > maxLen) {
                maxLen = len;
                maxLenAngle = Math.atan2(p2.x - p1.x, p2.y - p1.y) / Math.PI * 180 + 90;
            }
            Imgproc.line(frame, vertices[i], vertices[(i+1)%4], new Scalar(0, 0, 255), 2);
        }

        yellowSamplePose = new Pose(rect.center.x, rect.center.y, maxLenAngle);

        telemetry.addData("x", yellowSamplePose.x);
        telemetry.addData("y", yellowSamplePose.y);
        telemetry.addData("theta", yellowSamplePose.theta);

        return null;
    }

    public Pose getYellowSamplePose() {
        return yellowSamplePose;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
