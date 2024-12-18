package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;

public class RectDrawer extends OpenCvPipeline {
    enum PixelColor {
        WHITE(new Scalar(0, 0, 178.5), new Scalar(255, 26.9, 255), true),
        YELLOW(new Scalar(19, 50, 0), new Scalar(80, 225.0, 255.0), true),
        PURPLE(new Scalar(124.7, 36.8, 58.1), new Scalar(151.6, 117.6, 242.3), true),
        GREEN(new Scalar(38.3, 66.6, 83.6), new Scalar(55.3, 174.3, 255.0), true);
        public final Scalar UPPER;
        public final Scalar LOWER;
        public final boolean display;
        PixelColor(Scalar lower, Scalar upper, boolean display) {
            this.UPPER = upper; this.LOWER = lower; this.display = display;
        }
    }

    public static PixelColor[] colors = new PixelColor[]{PixelColor.WHITE, PixelColor.YELLOW, PixelColor.PURPLE, PixelColor.GREEN};

    public Mat frame;
    public int getCameraWidth() {
        return frame.width();
    }

    public static Telemetry telemetry;

    private int centX = 0;
    private int centY = 0;

    public Point getPixelsCenter() {
        return new Point(centX, centY);
    }

    public static Scalar testVariable = new Scalar(0, 1, 2);
    public static Scalar testVariable2 = new Scalar(0, 1, 2);

    public static Scalar lowerYellow = new Scalar(12.8, 131, 137.8); // hsv
    public static Scalar upperYellow = new Scalar(22.7, 255, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(114.8, 68, 32.6); // hsv
    public static Scalar upperBlue = new Scalar(136, 255, 255.0); // hsv
    public static Scalar lowerRed= new Scalar(161.5, 103.4, 82.2); // hsv
    public static Scalar upperRed = new Scalar(182.8, 255, 255.0); // hsv
    public static Scalar lowerTarget = new Scalar(114.8, 68, 32.6); // hsv
    public static Scalar upperTarget = new Scalar(136, 255, 255.0); // hsv

    public String colorType = "e";

//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }

    public RectDrawer(Telemetry telemetry){
        this.telemetry = telemetry;
//        this.colorType = colorType;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame = input;
        Mat output = new Mat();
        Mat hsv = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
//        Core.inRange(hsv, PixelColor.YELLOW.LOWER, PixelColor.YELLOW.UPPER, inRange);
        if (colorType.equals("blue")) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals("red")) {
            Core.inRange(hsv, lowerRed, upperRed, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }

        // inRange is the Binary mask

        // Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(14, 14));
        Imgproc.dilate(inRange, inRange, kernel);
        Imgproc.erode(inRange, inRange, kernel);

        // Step 1: Find non-zero points
        Mat nonZeroPoints = new Mat();
        Core.findNonZero(inRange, nonZeroPoints);

        // Step 2: Convert to MatOfPoint2f
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();
        if (!nonZeroPoints.empty()) {
            // Convert non-zero points (CV_32SC2) to CV_32FC2
            nonZeroPoints.convertTo(matOfPoint2f, CvType.CV_32FC2);
        }



        RotatedRect rect = Imgproc.minAreaRect(matOfPoint2f); // warning: may need try/catch statement

        Imgproc.cvtColor(inRange, inRange, Imgproc.COLOR_GRAY2RGBA);
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2BGR);
        Core.bitwise_and(input, inRange, output);

        Point[] boxPoints = new Point[4];
        rect.points(boxPoints);
        //Convert points to a MatOfPoint for drawing
        MatOfPoint box = new MatOfPoint();
        box.fromArray(boxPoints);
        Imgproc.drawContours(output, Arrays.asList(box), 0, new Scalar(0, 0, 255), 2);


        telemetry.addData("ee", inRange.type());
        telemetry.addData("input", input.type());
        telemetry.addData("center ", rect.center);
        telemetry.addData("size ", rect.size);
        telemetry.addData("angle ", rect.angle);
        telemetry.update();


        return output;


//        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(15, 15));
//        for (PixelColor c : colors) {
//            if (c != PixelColor.YELLOW) {
//                continue;
//            }
//            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);
//        }
//        return inRange;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }
    public MatOfPoint2f convertMatToMatOfPoint2f(Mat mat) {
        // Check if the Mat is in the correct format (CV_32FC2)
        if (mat.type() != CvType.CV_32FC2) {
            throw new IllegalArgumentException("Mat must be of type CV_32FC2");
        }

        // Create a MatOfPoint2f object
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();

        // Convert Mat rows to Point objects
        Point[] points = new Point[(int) mat.total()];
        for (int i = 0; i < mat.rows(); i++) {
            float[] data = new float[2];
            mat.get(i, 0, data);
            points[i] = new Point(data[0], data[1]);
        }

        // Set points to MatOfPoint2f
        matOfPoint2f.fromArray(points);

        return matOfPoint2f;
    }
}