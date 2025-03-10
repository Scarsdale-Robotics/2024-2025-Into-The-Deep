package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class gaussian extends OpenCvPipeline {

    public static double FIELD_RESOLUTION = 0.5; // inches
    public static double DECAY_TIME = 0.5;
    public static final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);
    public static double dist_covariance = 0.25;
    public static double PEAK_THRESHOLD = 0.9;


    public Mat yellowDistribution;
    public Mat blueDistribution;
    public Mat redDistribution;
    private final Mat gaussian2D;
    private int ksize;

    public static Telemetry telemetry;

    public gaussian(Telemetry telemetry){
        this.telemetry = telemetry;

        // Initialize an n x n single-channel matrix filled with zeros
        yellowDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);
        blueDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);
        redDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);

        // Initialize gaussian dist
        double sigma = Math.sqrt(dist_covariance)/FIELD_RESOLUTION;  // Standard deviation for Gaussian
        ksize = (int) Math.max(3, 6 * sigma);  // Kernel size (typically 6*sigma for near-zero tail)
        if (ksize % 2 == 0) ksize += 1;  // Ensure odd size for symmetry

        // Generate 1D Gaussian Kernels
        Mat kernelX = Imgproc.getGaussianKernel(ksize, sigma, CvType.CV_64F);
        Mat kernelY = Imgproc.getGaussianKernel(ksize, sigma, CvType.CV_64F);

        // Compute 2D Gaussian by outer product
        gaussian2D = new Mat();
        Core.gemm(kernelY, kernelX.t(), 1, new Mat(), 0, gaussian2D); // Outer product\
        Core.normalize(gaussian2D, gaussian2D, 0, 0.5, Core.NORM_MINMAX);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Decay current probabilities
        double deltaTime = 1.0/30.0;
        // Half life formula
        double decay = Math.pow(0.5, deltaTime/Math.abs(DECAY_TIME));
        yellowDistribution.convertTo(yellowDistribution, -1, decay, 0);
        blueDistribution.convertTo(blueDistribution, -1, decay, 0);
        redDistribution.convertTo(redDistribution, -1, decay, 0);


        // Apply Gaussian at center (r0, c0)
        int[][] coordinates = new int[][] {
                {0,RESOLUTION_N-10}
                ,{85,80}
                ,{78,79}
                ,{85,80}
                ,{78,79}
                ,{(int)(40+80*Math.random()),(int)(140+80*Math.random())}
                ,{(int)(RESOLUTION_N*Math.random()),(int)(RESOLUTION_N*Math.random())}
                ,{(int)(100+4*Math.random()),(int)(100+4*Math.random())}
                ,{RESOLUTION_N+1, RESOLUTION_N+1}
        };
        for (int[] coordinate : coordinates) {
            int r0 = coordinate[0];
            int c0 = coordinate[1];
            addGaussianToMat(yellowDistribution, r0, c0);
            addGaussianToMat(blueDistribution, r0, c0);
            addGaussianToMat(redDistribution, r0, c0);
        }


        // Find local maxima
        List<Point> yellowPeaks = findLocalMaxima(yellowDistribution, PEAK_THRESHOLD);
        List<Point> bluePeaks = findLocalMaxima(blueDistribution, PEAK_THRESHOLD);
        List<Point> redPeaks = findLocalMaxima(redDistribution, PEAK_THRESHOLD);

        telemetry.addData("yellowPeaks", yellowPeaks);
        telemetry.addData("bluePeaks", bluePeaks);
        telemetry.addData("redPeaks", redPeaks);
        telemetry.update();

        // Normalize to 8-bit range for display
        Mat viewable = new Mat();
        Core.add(yellowDistribution, blueDistribution, viewable);
        Core.add(viewable, redDistribution, viewable);
        Core.normalize(viewable, viewable, 0, 255, Core.NORM_MINMAX);
        viewable.convertTo(viewable, CvType.CV_8U);  // Convert to 8-bit grayscale
        Imgproc.cvtColor(viewable, viewable, Imgproc.COLOR_GRAY2BGR);
        for (Point peak : yellowPeaks) {
            Imgproc.circle(viewable,peak,1,new Scalar(0,0,255),-1);
        }

        return viewable;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }

    public List<Point> findLocalMaxima(Mat img, double threshold) {
        Mat binary = new Mat();
        Imgproc.threshold(img, binary, threshold, 255, Imgproc.THRESH_BINARY);
        binary.convertTo(binary, CvType.CV_8UC1);  // Convert to 8-bit

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Point> peaks = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Moments m = Imgproc.moments(contour);
            if (m.m00 != 0) {
                int cx = (int) (m.m10 / m.m00);
                int cy = (int) (m.m01 / m.m00);
                peaks.add(new Point(cx, cy));
            }
        }
        return peaks;
    }


    public void addGaussianToMat(Mat mat, int r0, int c0) {
        // Define the region to add the Gaussian
        int rStart = Math.max(0, r0 - ksize / 2);
        int rEnd = Math.min(mat.rows(), r0 + ksize / 2 + 1);
        int cStart = Math.max(0, c0 - ksize / 2);
        int cEnd = Math.min(mat.cols(), c0 + ksize / 2 + 1);

        // Compute corresponding submatrix in the Gaussian kernel
        int gRStart = rStart - (r0 - ksize / 2);  // Shifted row start in gaussian2D
        int gREnd = gRStart + (rEnd - rStart);    // Match size of the region in mat
        int gCStart = cStart - (c0 - ksize / 2);  // Shifted col start in gaussian2D
        int gCEnd = gCStart + (cEnd - cStart);    // Match size of the region in mat

        Mat roi = mat.submat(rStart, rEnd, cStart, cEnd);
        Mat subGaussian = gaussian2D.submat(gRStart, gREnd, gCStart, gCEnd);
        Core.add(roi, subGaussian, roi);
        Core.min(mat, new Scalar(1), mat);
    }


}