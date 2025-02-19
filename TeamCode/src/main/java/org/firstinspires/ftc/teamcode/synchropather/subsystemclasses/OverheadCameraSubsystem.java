package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor.SampleColor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class OverheadCameraSubsystem {

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    private final SampleOrientationProcessor processor;
    private final VisionPortal visionPortal;
    private boolean processorEnabled = false;

    private final Telemetry telemetry;

    // (X,Y)
    // +X is forward, +Y is to the left
    // How far forward the claw (at pickup position) is from the camera, in inches
    public static double[] CLAW_OFFSET = new double[]{-4.2, 0}; // TODO: TUNE THIS

    // How far forward and to the left the camera is from the robot's center of rotation
    // when the extendo is fully retracted, in inches.
    public static double[] CAMERA_OFFSET = new double[]{7.321, 0};//-0.57047244};


    public OverheadCameraSubsystem(WebcamName cameraName, Telemetry telemetry) {
        this.telemetry = telemetry;

        // init VisionPortal
        processor = new SampleOrientationProcessor(this.telemetry);
        visionPortal = buildVisionPortal(cameraName);
    }

    private VisionPortal buildVisionPortal(WebcamName cameraName) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessors(processor)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(processor, true);  // let processors run asynchronously using camera data
        processorEnabled = true;

        return visionPortal;
    }

    /**
     * Enables processor and turns on camera.
     */
    public void enableSampleOrientationProcessor() {
        visionPortal.setProcessorEnabled(processor, true);
        processorEnabled = true;
    }

    /**
     * Disables processor and turns off camera.
     */
    public void disableSampleOrientationProcessor() {
        visionPortal.setProcessorEnabled(processor, false);
        processorEnabled = false;
    }

    /**
     * @return {{X,Y}, {X,Y}, ...} where +X is forward and +Y is to the left
     */
    public ArrayList<double[]> getSamplePositions() {
        if (!processorEnabled) return new ArrayList<>();
        return processor.getRealPositions();
    }

    /**
     * @return radians, 0 means the sample is vertical
     */
    public ArrayList<Double> getSampleAngles() {
        if (!processorEnabled) return new ArrayList<>();
        return processor.getSampleAngles();
    }

    /**
     * @return {X,Y,H} of the sample that is nearest to the claw.
     */
    public double[] getClosestSample() {
        if (!processorEnabled) return null;
        ArrayList<double[]> samplePositions = getSamplePositions();
        ArrayList<Double> sampleAngles = getSampleAngles();
        if (samplePositions.isEmpty() || sampleAngles.isEmpty()) return null;

        // iterate through samples list
        double bestDistance = Double.POSITIVE_INFINITY;
        int bestIndex = -1;
        for (int i = 0; i < samplePositions.size(); i++) {
            double[] position = samplePositions.get(i);
            double distance = Math.hypot(
                    position[0] - CLAW_OFFSET[0],
                    position[1] - CLAW_OFFSET[1]
            );
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = i;
            }
        }

        return new double[]{
                samplePositions.get(bestIndex)[0],
                samplePositions.get(bestIndex)[1],
                sampleAngles.get(bestIndex)
        };
    }

    /**
     * @param color the sample color to detect
     */
    public void setFilterColor(SampleColor color) {
        if (!processorEnabled) return;
        processor.setFilterColor(color);
    }

    /**
     * Automatically adjusts camera exposure based on image brightness
     */
    public void correctExposure(LinearOpMode opMode, Telemetry telemetry) {
        if (!processorEnabled) return;
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) &&
                (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING ||
                processor.getAverageBrightness() == -1)) {
            telemetry.addData("[LOGI CAM] status", "waiting to start");
            telemetry.update();
        }

        // wait a bit
        ElapsedTime runtime = new ElapsedTime(0);
        runtime.reset();
        double delay = 0.25; // seconds
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && runtime.seconds() < delay);

        // finally update exposure
        updateExposure(visionPortal, getCorrectedExposure(processor.getAverageBrightness()));
        telemetry.addData("[LOGI CAM] status", "corrected exposure!");
        telemetry.update();
    }

    private void updateExposure(VisionPortal visionPortal, long ms) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(ms, TimeUnit.MILLISECONDS);  // exposure may have to be adjusted during competitions
    }

    private long getCorrectedExposure(double averageBrightness) {
        if (averageBrightness < 50) return 50;
        else if (averageBrightness < 80) return 27;
        else if (averageBrightness < 100) return 15;
        else if (averageBrightness < 140) return 14;
        else return 5;
    }

}
