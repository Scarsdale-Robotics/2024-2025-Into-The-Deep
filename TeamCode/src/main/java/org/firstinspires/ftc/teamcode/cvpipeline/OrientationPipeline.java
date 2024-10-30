package org.firstinspires.ftc.teamcode.cvpipeline;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OrientationPipeline /*implements VisionProcessor*/ extends OpenCvPipeline {
    Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(6, 6));
    List<Mat> dataMats = new ArrayList<>();
    public Scalar redLower = new Scalar(0,100,150);
    public Scalar redUpper = new Scalar(10,255,255);
    public Scalar redLower2 = new Scalar(165,100,150);
    public Scalar redUpper2 = new Scalar(180,255,255);
    public Scalar greenLower = new Scalar(50,100,150);
    public Scalar greenUpper = new Scalar(100,255,255);
    public Scalar yellowLower = new Scalar(13,65,120);
    public Scalar yellowUpper = new Scalar(27,255,255);
    public Scalar blueLower = new Scalar(100,100,50);
    public Scalar blueUpper = new Scalar(125,255,255);
    List<MatOfPoint> contours = new ArrayList<>();
    List<MatOfPoint> redcontours = new ArrayList<>();
    List<MatOfPoint> bluecontours = new ArrayList<>();
    List<MatOfPoint> yellowcontours = new ArrayList<>();

    Mat redpixels = new Mat();
    Mat yellowpixels = new Mat();
    Mat greenpixels = new Mat();
    Mat redpixels2 = new Mat();
    Mat yellowpixels2 = new Mat();
    Mat bluepixels2 = new Mat();
    Mat bluepixels = new Mat();
    Mat colorpixels = new Mat();
    public Scalar contColor = new Scalar(0,255,255);
    public Scalar thress = new Scalar(20,75);
    public int showColor = 0;
    Mat result = new Mat();
    Mat HSV = new Mat();
    Mat H = new Mat();
    public List<MatOfPoint> yellowContours = new ArrayList<>();
    List<Mat> outlines = new ArrayList<>();
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }
    Telemetry telemetry;
    public OrientationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat frame) {
        result= frame.clone();
        Core.max(CVhelpers.getcolormat(frame,redLower,redUpper,kernel),CVhelpers.getcolormat(frame,redLower2,redUpper2,kernel),redpixels);


        yellowpixels = CVhelpers.getcolormat(frame,yellowLower,yellowUpper,kernel);
//        greenpixels = CVhelpers.getcolormat(frame,greenLower,greenUpper);
        bluepixels = CVhelpers.getcolormat(frame,blueLower,blueUpper,kernel);
        telemetry.addData("type", frame.channels());
        telemetry.update();

        Imgproc.cvtColor(redpixels,redpixels2,Imgproc.COLOR_GRAY2RGBA);
        Imgproc.cvtColor(yellowpixels,yellowpixels2,Imgproc.COLOR_GRAY2RGBA);
        Imgproc.cvtColor(bluepixels,bluepixels2,Imgproc.COLOR_GRAY2RGBA);

        Core.bitwise_and(redpixels2,frame,redpixels2);
        Core.bitwise_and(yellowpixels2,frame,yellowpixels2);
        Core.bitwise_and(bluepixels2,frame,bluepixels2);

        Core.add(redpixels2,yellowpixels2,colorpixels);
        Core.add(colorpixels,bluepixels2,colorpixels);


        redcontours= CVhelpers.findGrayContours(redpixels);
        yellowcontours = CVhelpers.findGrayContours(yellowpixels);
        bluecontours =CVhelpers.findGrayContours(bluepixels);

        //combine into one list
        contours = redcontours;
        contours.addAll(yellowcontours);
        contours.addAll(bluecontours);
        Imgproc.drawContours(colorpixels, contours, -1, contColor);
        H = CVhelpers.getFixedHueOnlyMat(frame);
        Imgproc.Canny(H,result,thress.val[1],thress.val[2]);
        return colorpixels;
    }

//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

//    }
}
