package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
@TeleOp(name="Sample Localization Logger", group="Calibration")
public class SampleLocalizationLogger extends LinearOpMode {

    private Limelight3A limelight;

    // Sample pose estimation coefficients
    public static double k1 = 1;
    public static double k2 = 1;
    public static double cz = 4; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
        this.limelight = hardwareRobot.limelight;

        // Switch to detector
        this.limelight.close();
        this.limelight.pipelineSwitch(2);
        this.limelight.start();

        waitForStart();

        Set<Color> colors = new HashSet<>(
                Arrays.asList(Color.BLUE, Color.RED, Color.YELLOW)
        );

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            List<DetectorResult> detections = result.getDetectorResults();
            List<DetectorResult> selectedDetections = new ArrayList<>();

            // Filter out samples
            for (DetectorResult detection : detections) {
                String className = detection.getClassName();
                Color detectedColor;
                switch (className) {
                    case "red":
                        detectedColor = Color.RED;
                        break;
                    case "blue":
                        detectedColor = Color.BLUE;
                        break;
                    default:
                        detectedColor = Color.YELLOW;
                }

                telemetry.addData("CL", className);

                if (colors.contains(detectedColor)) selectedDetections.add(detection);
            }

            if (!selectedDetections.isEmpty()) {
                telemetry.addData("_", "SAMPLE DETECTED");

                // Log corners of each sample
                for (int i = 0; i < selectedDetections.size(); i++) {
                    DetectorResult detection = selectedDetections.get(i);
                    telemetry.addData(String.format("[%s].corners", i), detection.getTargetCorners());

                    // Draw position of sample
                    Pose2d samplePoseEstimation = calculateSampleRelativePosition(detection, cz);
                    if (samplePoseEstimation != null) {
                        TelemetryPacket packet = new TelemetryPacket();
                        packet.fieldOverlay().setStroke("#3F51B5");
                        Drawing.drawRobot(packet.fieldOverlay(), samplePoseEstimation);
                        FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    }
                }

                // Telemeter position of first sample
                DetectorResult detection = selectedDetections.get(0);
                Pose2d samplePoseEstimation = calculateSampleRelativePosition(detection, cz);
                if (samplePoseEstimation != null) {
                    telemetry.addData("[0]SAMPLE X", samplePoseEstimation.getX());
                    telemetry.addData("[0]SAMPLE Y", samplePoseEstimation.getY());
                }
            } else {
                telemetry.addData("_", "NO DETECTION");
            }

            telemetry.update();


        }



    }

    /**
     * Estimates the sample's position relative to the robot given the midpoint of its bounding box's top edge.
     *
     * @param detection The Limelight3A detector result for this sample.
     * @param cz Z position of the camera relative to the ground in inches.
     * @return the estimated position.
     */
    private Pose2d calculateSampleRelativePosition(DetectorResult detection, double cz) {

        // Get pixel coordinates of the midpoint of the bounding box's top edge.
        List<List<Double>> corners = detection.getTargetCorners();
        double px = 0.5 * (corners.get(0).get(0) + corners.get(1).get(0));
        double py = 0.5 * (corners.get(0).get(1) + corners.get(1).get(1));

        // k1*x' = -y/x
        // y' = k2*z/x
        //
        // x = k2*z/y'
        // y = -k1*x*x' = -k1*k2*x'*z/y'

        if (py == 0) {
            return null;
        }

        double projectedX = (k2 * cz) / py;
        double projectedY = -(k1 * k2 * px * cz) / py;

        return new Pose2d(projectedX, projectedY, new Rotation2d(Math.toRadians(0)));
    }

    public enum Color { RED, YELLOW, BLUE }

}
