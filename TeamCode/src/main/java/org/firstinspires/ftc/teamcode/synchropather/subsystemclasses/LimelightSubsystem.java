package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.cvprocessors.LimelightDetectorProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightState;

import java.util.ArrayList;
import java.util.List;

public class LimelightSubsystem {

    public final Limelight3A limelight;
    public final LimelightDetectorProcessor processor;
    private LimelightState currentState;

    private List<double[]> yellowSamplePositions;
    private List<double[]> blueSamplePositions;
    private List<double[]> redSamplePositions;


    public LimelightSubsystem(Limelight3A limelight, LimelightDetectorProcessor processor) {
        this.limelight = limelight;
        this.processor = processor;
        this.currentState = null;
        this.yellowSamplePositions = new ArrayList<>();
        this.blueSamplePositions = new ArrayList<>();
        this.redSamplePositions = new ArrayList<>();
    }

    public void startLimelight() {
        limelight.start();
    }

    public void pauseLimelight() {
        limelight.pause();
    }

    public void switchLimelightPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public List<double[]> getYellowSamplePositions() {
        return yellowSamplePositions;
    }

    public List<double[]> getBlueSamplePositions() {
        return blueSamplePositions;
    }

    public List<double[]> getRedSamplePositions() {
        return redSamplePositions;
    }

    public void setCurrentState(LimelightState currentState) {
        this.currentState = currentState;
    }

    public void update(SampleDataBufferFilter limelightSampleData) {
        if (currentState.getPipeline()==null || !currentState.getEnabled() || !limelight.isRunning()) return;

        // Pipeline-specific actions
        if (currentState.getPipeline() == LimelightPipeline.SAMPLE_DETECTOR) {
            // Add new detections to the processor
            List<LLResultTypes.DetectorResult> detections = limelight.getLatestResult().getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                double[] relativePosition = calculateSampleRelativePosition(detection, LimelightConstants.FOV, LimelightConstants.cz, LimelightConstants.theta_incline);
                if (relativePosition != null) {
                    double[] fieldPosition = calculateFieldPosition(limelightSampleData.getLastBufferedBotPose(), relativePosition);
                    processor.addSamplePosition(fieldPosition);
                }
            }
        }

        // Update sample positions
        yellowSamplePositions = processor.getYellowSamplePositions();
        blueSamplePositions = processor.getBlueSamplePositions();
        redSamplePositions = processor.getRedSamplePositions();
    }

    /**
     * Estimates the sample's position relative to the robot given the midpoint of its bounding box's top edge.
     *
     * @param detection The Limelight3A detector result for this sample.
     * @return the estimated position.
     */
    private double[] calculateSampleRelativePosition(LLResultTypes.DetectorResult detection, double FOV, double cz, double theta_incline) {

        // math: https://docs.google.com/document/d/1hrrhX6M2sVAjVzJZFjrWjbrTWbZSyLkcwsaZOKVoI-A/edit?usp=sharing

        // Get pixel coordinates of midpoint of top edge of bounding box.
        List<List<Double>> corners = detection.getTargetCorners();
        double px = 0.5 * (corners.get(0).get(0) + corners.get(1).get(0));
        double py = corners.get(0).get(1);

        // Back-solve for projected coordinates
        double c_px = (px - 320) / FOV;
        double c_py = (240 - py) / FOV;

        double projectedX_numer = -c_py*Math.sin(theta_incline) - Math.cos(theta_incline);
        double projectedX_denom = c_py*Math.cos(theta_incline) - Math.sin(theta_incline);
        if (projectedX_denom == 0) return null;

        double projectedX = (cz-1.5) * projectedX_numer / projectedX_denom;
        double projectedY = -(projectedX*Math.cos(theta_incline) + (cz-1.5)*Math.sin(theta_incline)) * c_px;

        // These color codes are according to LimelightDetectorProcessor
        // 0: yellow
        // 1: blue
        // 2: red
        int detectedColor;
        switch (detection.getClassName()) {
            case "yellow":
                detectedColor = 0;
                break;
            case "blue":
                detectedColor = 1;
                break;
            case "red":
                detectedColor = 2;
                break;
            default:
                detectedColor = -1;
                break;
        }

        return new double[]{
                projectedX,
                projectedY,
                detectedColor
        };
    }


    /**
     * Calculates the global position of an object given its relative pose to the robot and the robot's Pose.
     * @param robotPose
     * @param relativePosition
     * @return the object's global Pose2d.
     */
    private double[] calculateFieldPosition(Pose2d robotPose, double[] relativePosition) {
        Pose2d limelightPosition = LimelightConstants.limelightPosition;
        double[] globalPose = relativePosition;
        double dTheta;
        Translation2d dTranslation;

        // correct for limelight heading
        dTheta = limelightPosition.getHeading();
        globalPose = rotatePose(globalPose, dTheta);

        // correct for limelight translation
        dTranslation = limelightPosition.getTranslation();
        globalPose = translatePosition(globalPose, dTranslation);

        // correct for robot heading
        dTheta = robotPose.getHeading();
        globalPose = rotatePose(globalPose, dTheta);

        // correct for robot translation
        dTranslation = robotPose.getTranslation();
        globalPose = translatePosition(globalPose, dTranslation);

        return globalPose;
    }


    /**
     * Rotates the given position about the origin by the given angular displacement.
     * @param position
     * @param dTheta in radians
     * @return the rotated position.
     */
    private double[] rotatePose(double[] position, double dTheta) {
        double x = position[0];
        double y = position[1];
        double cos = Math.cos(dTheta);
        double sin = Math.sin(dTheta);

        return new double[] {
                x*cos - y*sin,
                x*sin + y*cos,
                position[2]
        };
    }


    /**
     * Translates the given position by the given displacement.
     * @param position
     * @param translation
     * @return the translated position.
     */
    private double[] translatePosition(double[] position, Translation2d translation) {
        return new double[] {
                position[0] + translation.getX(),
                position[1] + translation.getY(),
                position[2]
        };
    }


}
