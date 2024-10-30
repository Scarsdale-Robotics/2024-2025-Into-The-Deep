package org.firstinspires.ftc.teamcode.cvpipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class CVhelpers {
    public static Mat getGrayFilteredRGBMat(Mat input){
        Mat result = input;
        for (int row = 0; row <input.height();row ++){for(int collumn = 0; collumn<input.width(); collumn++){
            double[] data = input.get(row, collumn);
            double minimum = Math.min(data[0], Math.min(data[1],data[2]));
            double maximum = Math.max(data[0], Math.max(data[1],data[2]));
            if (maximum-minimum<30){
                result.put(row,collumn,new double[]{0,0,0,0});
            }
        }}
        return result;
    }
    public static Mat getcolormat(Mat input, Scalar lower, Scalar upper,Mat kernel){

        //convert input to an HSV mat saved as "out"
        Mat out=new Mat();
        Imgproc.cvtColor(input,out,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(out, out, Imgproc.COLOR_RGB2HSV);
        Imgproc.morphologyEx(out,out,Imgproc.MORPH_CLOSE,kernel);
//        Imgproc.morphologyEx(out,out,Imgproc.MORPH_OPEN,kernel);
        //convert out to a new mat where each pixel is pure white if the corresponding pixel on the input is within the
        //constraints, and pure black otherwise
        Core.inRange(out, lower, upper, out);
//        Imgproc.cvtColor(out,out,Imgproc.COLOR_GRAY2RGBA);
        //return said Mat
        return out;
    }

    public static List<Mat> getMatPixelStats(Mat input){
        List<Mat> result = new ArrayList<>();
        Mat HSV = new Mat();
        Imgproc.cvtColor(input,HSV,Imgproc.COLOR_RGB2HSV);
        int width = input.width();
        int height = input.height();
        Mat satMat = new Mat(height,width,0);
        Mat valueMat = new Mat(height,width,0);
        Mat hueMat = getFixedHueOnlyMat(HSV);
        for (int row = 0; row <input.height();row ++){for(int collumn = 0; collumn<input.width(); collumn++){
            double[] data = input.get(row,collumn);
            double[] hsvdata = HSV.get(row,collumn);
            valueMat.put(row,collumn,hsvdata[2]);
            satMat.put(row,collumn,hsvdata[1]);
        }}
        result.add(hueMat);
        result.add(satMat);
        result.add(valueMat);
        return result;


    }
    public static List<Mat> getAllOutlines(Mat input,double upthres,double downthres){
        List<Mat> Outlines = new ArrayList<>();

        for(int type = 0; type<3;type ++){
            Mat temp = new Mat();
            if(type == 1) {
//                List<Mat> dataMats = getMatPixelStats(input);
//                Imgproc.Canny(dataMats.get(type), temp, upthres, downthres);
            }
            if(type == 2){
                temp = getGrayFilteredRGBMat(input);
                temp = valueOutlines(temp,upthres,downthres);
                Imgproc.cvtColor(temp,temp,Imgproc.COLOR_GRAY2RGBA);

            }
            if (type == 0){
                temp = hueOutlines(input,upthres,downthres);
                Imgproc.cvtColor(temp,temp,Imgproc.COLOR_GRAY2RGBA);

            }
            Outlines.add(temp);
        }
        return Outlines;
    }
    public static Mat hueOutlines(Mat input, double upthres,double downthres){
        Mat cannyResult = new Mat();
        Mat HSV = new Mat();
        HSV = getFixedHueMat(input);
        Mat HSVGray = new Mat();
        Mat loopedResult = new Mat();
        Mat temp = new Mat();
        Imgproc.cvtColor(HSV,HSVGray,Imgproc.COLOR_RGB2GRAY);
        Mat HSVlooped = new Mat();
        Imgproc.cvtColor(HSV,HSVlooped,Imgproc.COLOR_HSV2RGB);
        for (int row = 0; row <HSV.height();row ++){for(int collumn = 0; collumn<HSV.width(); collumn++){
            double[] data = HSV.get(row,collumn);
            int loop = 90;
            data[0] = data[0]<loop ?data[0]-loop :data[0]+loop;
//            data
            HSVlooped.put(row,collumn,data);
        }}
        if (HSVlooped.empty()) {
            return HSVGray;
        }
        Imgproc.cvtColor(HSVlooped,temp, Imgproc.COLOR_RGB2GRAY);
//        if(HSVlooped.size() != HSVGray)
        Imgproc.Canny(HSVGray, cannyResult, upthres, downthres);
        Imgproc.Canny(HSVlooped,loopedResult, upthres,downthres);
//        Core.bitwise_not(cannyResult,temp);
        Core.bitwise_and(cannyResult,loopedResult,temp);

        return cannyResult;

    }
    public static Mat valueOutlines(Mat input, double upthres,double downthres){
        Mat cannyResult = new Mat();
        Mat gray = new Mat();
        Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(gray, cannyResult, upthres, downthres);
        return cannyResult;
    }
    public static Mat getFixedHueMat(Mat input){
        Mat result = new Mat();
        Imgproc.cvtColor(input,result, Imgproc.COLOR_RGB2HSV);
        int height = result.height();
        int width = result.width();
        for (int row = 0; row <height;row ++){for(int collumn = 0; collumn<width; collumn++){
            double[] data = input.get(row, collumn);
            double minimum = Math.min(data[0], Math.min(data[1],data[2]));
            double maximum = Math.max(data[0], Math.max(data[1],data[2]));
            if (maximum-minimum<40){
                result.put(row,collumn,new double[]{400,0,0});
            }
        }}
        return result;
    }
    public static Mat getFixedHueOnlyMat(Mat input){
        Mat result = new Mat();
        Mat HSV = new Mat();
        Imgproc.cvtColor(input,HSV,Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input,result, Imgproc.COLOR_RGB2GRAY);
        int height = result.height();
        int width = result.width();
        for (int row = 0; row <height;row ++){for(int collumn = 0; collumn<width; collumn++){
            double[] data = input.get(row, collumn);
            double minimum = Math.min(data[0], Math.min(data[1],data[2]));
            double maximum = Math.max(data[0], Math.max(data[1],data[2]));
            if (maximum-minimum<40){
                result.put(row,collumn,400);
            }
            else{
                result.put(row,collumn,HSV.get(row,collumn));
            }
        }}
        return result;
    }
    public static Mat getFixedHSV(Mat input){
        Mat result = new Mat();
        Imgproc.cvtColor(input,result, Imgproc.COLOR_RGB2HSV);

        int height = result.height();
        int width = result.width();
        for (int row = 0; row <height;row ++){for(int collumn = 0; collumn<width; collumn++){
            double[] data = input.get(row, collumn);
            double[] resultdata = result.get(row,collumn);
            double minimum = Math.min(data[0], Math.min(data[1],data[2]));
            double maximum = Math.max(data[0], Math.max(data[1],data[2]));
            if (maximum-minimum<30){
                result.put(row,collumn,new double[]{400,0,resultdata[2]});
            }
        }}
        return result;
    }
    public static List<MatOfPoint> findGrayContours(Mat input){
        List<MatOfPoint> countours = new ArrayList<>();
        Mat heirarchy = new Mat();
        Imgproc.findContours(input,countours,heirarchy,Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return countours;
    }
    public static List<MatOfPoint> getColoredContours(Mat input, Scalar lower, Scalar upper, Scalar lower2, Scalar upper2,Mat kernel) {
        Mat contourResult = getcolormat(input,lower,upper,kernel);
        Mat contourResult2 = getcolormat(input,lower2,upper2,kernel);
        Imgproc.cvtColor(contourResult2,contourResult2,Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(contourResult,contourResult,Imgproc.COLOR_RGB2GRAY);
        Mat contourhierarchy = new Mat();
        List<MatOfPoint> contourList = new ArrayList<>();
        Core.max(contourResult, contourResult2, contourResult);
        Imgproc.findContours(contourResult, contourList, contourhierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contourList;
    }

    public static List<MatOfPoint> getColoredContours(Mat input, Scalar lower, Scalar upper,Mat kernel) {
        Mat contourResult = getcolormat(input,lower,upper,kernel);
        Imgproc.cvtColor(contourResult,contourResult,Imgproc.COLOR_RGB2GRAY);
        Mat contourhierarchy = new Mat();
        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(contourResult, contourList, contourhierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contourList;
    }

    public static List<MatOfPoint> filterSmall(List<MatOfPoint> Contours, int minArea) {
        List<MatOfPoint> eligableContours = new ArrayList<>(); //not bachellors
        for (MatOfPoint contour : Contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea >= minArea) {
                eligableContours.add(contour);
            }

        }
        return eligableContours;
    }
}