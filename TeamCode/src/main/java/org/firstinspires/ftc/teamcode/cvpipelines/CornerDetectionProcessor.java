package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
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

public class CornerDetectionProcessor implements VisionProcessor {
    boolean isRedTeam;

    public Mat sub;
    private Mat output;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static int testVariable = 42;


    public AtomicBoolean hasStarted = new AtomicBoolean(false);

    private Mat rgbaMat = new Mat();
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    //    public static Size kernelSize = new Size(5, 5);
    public static double kernelSize1 = 5;
    public static double kernelSize2 = 5;

    // List to store detected boxes
    private List<MatOfPoint> boxes = new ArrayList<>();

    // Scalar values for color range (HSV) - Adjust these values based on lighting conditions
//    public Scalar lowerYellow = new Scalar(20, 100, 100);
//    public Scalar upperYellow = new Scalar(30, 255, 255);
    public static double lowerYellowR = 10;
    public static double lowerYellowG = 30;
    public static double lowerYellowB = 30;
    public static double upperYellowR = 255;
    public static double upperYellowG = 255;
    public static double upperYellowB = 255;


    public static double k_epsilon = 0.02;
//    private Scalar lowerYellow;
//    private Scalar upperYellow;

    private Telemetry telemetry;



    public CornerDetectionProcessor(Telemetry telemetry){
        this.telemetry = telemetry;

    }
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long nanoStart) {

        Scalar lowerYellow = new Scalar(lowerYellowR, lowerYellowG, lowerYellowB);
        Scalar upperYellow = new Scalar(upperYellowR, upperYellowG, upperYellowB);

        // Convert input image to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a binary mask where yellow colors are white and the rest are black
        Core.inRange(hsvMat, lowerYellow, upperYellow, mask);

        // Apply morphological operations to clean up the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelSize1, kernelSize2));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear previous boxes
        boxes.clear();

        // Loop over contours to find boxes
        for (MatOfPoint contour : contours) {
            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = k_epsilon * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            // Convert back to MatOfPoint
            MatOfPoint approxContour = new MatOfPoint(approxCurve.toArray());

            // Check if the approximated contour has 4 vertices and is convex
            if (approxContour.total() == 4 && Imgproc.isContourConvex(approxContour)) {
                // Compute the area to filter out small contours if needed
                double area = Imgproc.contourArea(approxContour);
                if (area > 1000) { // TODO: Adjust the area threshold as needed
                    // Add the detected box to the list
                    boxes.add(approxContour);

                    // Draw the contour on the input image
                    Imgproc.drawContours(input, Arrays.asList(approxContour), -1, new Scalar(0, 255, 0), 2);

                    // Draw circles at the vertices
                    Point[] vertices = approxContour.toArray();
                    for (Point vertex : vertices) {
                        Imgproc.circle(input, vertex, 5, new Scalar(255, 0, 0), -1);
                    }
                }
            }
        }

        // Update telemetry with the number of detected boxes
        telemetry.addData("Detected Contours", contours.size());
        telemetry.addData("Detected Boxes", boxes.size());
        telemetry.update();

        return input;
    }

    // Getter method to retrieve detected boxes
    public List<MatOfPoint> getDetectedBoxes() {
        return boxes;
    }

//    @Override
//    public void onViewportTapped() {
//        // Implement any functionality if needed when the viewport is tapped
//        return;
//    }

    // Release resources when done
    public void release() {
        hsvMat.release();
        mask.release();
        hierarchy.release();
        // Release any other Mats if created
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}