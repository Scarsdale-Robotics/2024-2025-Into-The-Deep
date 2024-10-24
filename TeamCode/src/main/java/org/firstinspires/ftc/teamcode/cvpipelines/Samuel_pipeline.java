package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.Locale;
import java.util.ArrayList;
import java.util.List;

public class Samuel_pipeline implements VisionProcessor {

    Telemetry telemetry; // telemetry
    public Samuel_pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert frame to HSV color space
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        // Defining HSV ranges for red (two ranges)
        Scalar lowerRed1 = new Scalar(0, 150, 150);
        Scalar upperRed1 = new Scalar(5, 255, 255);
        Scalar lowerRed2 = new Scalar(175, 150, 150);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Create masks for each red range
        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Core.inRange(frame, lowerRed1, upperRed1, redMask1);
        Core.inRange(frame, lowerRed2, upperRed2, redMask2);

        // Combine both masks
        Mat redMask = new Mat();
        Core.add(redMask1, redMask2, redMask);

        // Defining HSV ranges for green and yellow
        Scalar lowerGreen = new Scalar(45, 100, 100);
        Scalar upperGreen = new Scalar(80, 255, 255);
        Scalar lowerYellow = new Scalar(19, 70, 70);
        Scalar upperYellow = new Scalar(32, 255, 255);


        // create masks for Yellow
        Mat yellowMask = new Mat();
        Core.inRange(frame, lowerYellow, upperYellow, yellowMask);

        // create masks for Green
        Mat greenMask = new Mat();
        Core.inRange(frame, lowerGreen, upperGreen, greenMask);

        // below is the the watershed algorithm for yellow
        // convert yellow mask to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(yellowMask, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply a threshold to binarize the image
        Mat binary = new Mat();
        Imgproc.distanceTransform(gray, binary, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);

        // Perform distance transform
        Mat distTransform = new Mat();
        Imgproc.distanceTransform(binary, distTransform, Imgproc.DIST_L2, 5);

        // normalize the distance image for visualization
        Core.normalize(distTransform, distTransform, 0, 1.0, Core.NORM_MINMAX);

        // Threshold the distance transform image
        Mat distThreshold = new Mat();
        Imgproc.threshold(distTransform, distThreshold, 0.5, 1.0, Imgproc.THRESH_BINARY);

        // Dilate the distance transform image to fill holes
        Mat Kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.dilate(distThreshold, distThreshold, Kernel);

        // Find sure foreground and background areas
        Mat Markers = new Mat();
        Core.convertScaleAbs(distThreshold, Markers);

        // watershed algorithm
        Imgproc.watershed(frame, Markers);

        // creating yellow contours
        List<MatOfPoint> yellowContours = new ArrayList<>();
        Imgproc.findContours(Markers, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // creating a box for the object

        for (MatOfPoint contour : yellowContours) {

            // Get the bounding boc for each contour
            Rect boundingBox = Imgproc.boundingRect(contour);

            // drawing the bounding box on frame
            Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(255, 0, 0), 1);

            Point center = new Point(boundingBox.x + boundingBox.width / 2.0, boundingBox.y + boundingBox.height / 2.0);
            telemetry.addData("Center of Object", center.toString());
        }

        // Convert back to RGB before drawing contours
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);


        // drawing contour
        Imgproc.drawContours(frame, yellowContours, -1, new Scalar(128, 0, 128), 1);

        telemetry.update();

        return frame;  // Return the processed frame

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
