package org.firstinspires.ftc.teamcode.cvpipelines;


import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class theRealSam_pipeline implements VisionProcessor {

    Telemetry telemetry; // telemetry
    public theRealSam_pipeline (Telemetry telemetry) {
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

        // Defining HSV ranges for green and yellow and creating masks
        Scalar lowerBlue = new Scalar(100, 100, 20);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Scalar lowerYellow = new Scalar(13, 150, 60);
        Scalar upperYellow = new Scalar(33, 255, 255);

        Mat yellowMask = new Mat();
        Core.inRange(frame, lowerYellow, upperYellow, yellowMask);

        Mat blueMask = new Mat();
        Core.inRange(frame, lowerBlue, upperBlue, blueMask);

        Mat combinedMask = new Mat(); // combine all masks into one
        Core.add(redMask, yellowMask, combinedMask);
        Core.add(combinedMask, blueMask, combinedMask);

        // creating contours for all colors
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        // creating a gray scale so edge detection and vertex detection can be implemented
        Imgproc.cvtColor(combinedMask, combinedMask, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);
        Core.bitwise_and(frame, combinedMask, frame);
        Mat grayFrame = frame.clone();
        Imgproc.cvtColor(grayFrame, grayFrame, Imgproc.COLOR_RGB2GRAY);

        // cranny edge detection
        Mat edges = new Mat();
        double lowerThreshold = 80;
        double upperThreshold = 180;
        Imgproc.Canny(grayFrame, edges, lowerThreshold, upperThreshold);

        // drawing edges
        for (int i = 0; i < edges.rows(); i ++) {
            for (int j = 0; j < edges.cols(); j++) {
                if (Arrays.equals(edges.get(i, j), new double[]{255})) {
                    frame.put(i, j, new double[]{0, 0, 255});
                }
            }
        }
        Mat normalFrame = new Mat();
        Imgproc.cvtColor(edges, normalFrame, Imgproc.COLOR_GRAY2RGB);


        // apply Hough Line Transform
        Mat lines = new Mat(); // linesP means lines probabilistic
        Imgproc.HoughLines(edges, lines, 1, Math.PI / 180, 10, 30, 30); //rho, theta, threshold, minLineLength, maxLineGap

        //draw the detected lines
        for (int i = 0; i < lines.rows(); i++) {
            double rho = lines.get(i, 0)[0],
                    theta = lines.get(i, 0)[1];

            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
            Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
            Imgproc.line(normalFrame, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);

        }


        double minAreaThreshold = 90;
        for (int i = 0; i < contours.size(); i++) {
            double contourArea = Imgproc.contourArea(contours.get(i));
            if (contourArea < minAreaThreshold) {
                continue;
            }
            Rect rect = Imgproc.boundingRect(contours.get(i));
            Imgproc.rectangle(frame, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 1);
        }


        telemetry.update();

        return frame;  // Return the processed frame
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
