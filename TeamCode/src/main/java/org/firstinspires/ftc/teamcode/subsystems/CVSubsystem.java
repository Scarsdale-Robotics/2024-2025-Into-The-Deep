package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class CVSubsystem extends SubsystemBase {

    /**
     * PIPELINE INDICES:
     * 0 = AprilTag Pipeline
     */
    private final Limelight3A limelight;

    private final Telemetry telemetry;


    /**
     * Constructs a new CVSubsystem object with the given parameters.
     * @param limelight The Limelight3A object.
     * @param initialHeading The robot's initial heading in radians.
     * @param telemetry The Telemetry object.
     */
    public CVSubsystem(Limelight3A limelight, double initialHeading, Telemetry telemetry) {
        this.limelight = limelight;
        this.limelight.updateRobotOrientation(Math.toDegrees(initialHeading));
        this.telemetry = telemetry;

        // Switch to AprilTag
        limelight.pipelineSwitch(0);
        limelight.start();

    }



    //////////////////
    // MAIN METHODS //
    //////////////////

    /**
     * Updates the Limelight's heading for the MT2 algorithm.
     * @param heading The robot heading in radians.
     */
    public void updateHeading(double heading) {
        limelight.updateRobotOrientation(Math.toDegrees(heading));
    }

    /**
     * Logs the robot poses calculated using the detected AprilTags relative to the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#reference-frame">field reference frame</a>.
     */
    public void telemeterAbsoluteAprilTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Position position = result.getBotpose_MT2().getPosition();
                telemetry.addData(
                        "LL Botpose",
                        String.format("(X:%s, Y:%s)", position.x, position.y)
                );
            }
        }
        telemetry.update();
    }

    /**
     * Chooses the AprilTag pose estimation with the least uncertainty.
     */
    public PoseEstimation getPoseEstimation() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;
        if (!result.isValid()) return null;

        Pose3D botPose = result.getBotpose_MT2();
        double[] deviations = result.getStddevMt2();


        // Splice translation and heading estimates.
        return new PoseEstimation(
                new Pose2d(botPose.getPosition().x, botPose.getPosition().y, new Rotation2d(botPose.getOrientation().getYaw(AngleUnit.RADIANS))),
                deviations[0] * deviations[0] + deviations[1] * deviations[1],
                deviations[2] * deviations[2]
        );
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        return (radians + Math.PI) % (2*Math.PI) - Math.PI;
    }

    /**
     * Contains the pose and uncertainties for an AprilTag pose estimation.
     */
    public static class PoseEstimation {
        public Pose2d pose;
        public double translationCovariance;
        public double headingCovariance;

        public PoseEstimation(Pose2d pose, double translationCovariance, double headingCovariance) {
            this.pose = pose;
            this.translationCovariance = translationCovariance;
            this.headingCovariance = headingCovariance;
        }
    }

}