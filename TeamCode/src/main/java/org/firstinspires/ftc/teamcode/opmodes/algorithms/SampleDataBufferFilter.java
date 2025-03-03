package org.firstinspires.ftc.teamcode.opmodes.algorithms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class SampleDataBufferFilter {

    public enum SampleTargetingMethod {
        ROTATION(),
        TRANSLATION()
    }

    private final LinearSlidesSubsystem linearSlides;
    private final LocalizationSubsystem localization;

    // For data buffering to account for the camera's lag
    private final ElapsedTime runtime;
    private final double timeBuffer;
    private final ArrayList<double[]> bufferedExtendoPositions;  // [{position, timestamp}, ...]
    private final ArrayList<Pose2d> bufferedBotPoses;
    private final ArrayList<Double> bufferedBotPoseTimes;
    private ExtendoState lastBufferedExtendoPosition;
    private Pose2d lastBufferedBotPose;

    // For filtering incoming cv sample data
    private final int filterLength;
    private final ArrayList<Pose2d> samplePoses; // {x_sample, y_sample, theta_sample}
    private final SampleTargetingMethod targetingMethod;

    public static double FILTER_ERROR_TOLERANCE = 0.25; // inches

    public SampleDataBufferFilter(LinearSlidesSubsystem linearSlides, LocalizationSubsystem localization, double timeBuffer, int filterLength, SampleTargetingMethod targetingMethod) {
        this.linearSlides = linearSlides;
        this.localization = localization;

        // init buffer
        runtime = new ElapsedTime(0);
        runtime.reset();
        this.timeBuffer = timeBuffer;
        bufferedExtendoPositions = new ArrayList<>();
        bufferedBotPoses = new ArrayList<>();
        bufferedBotPoseTimes = new ArrayList<>();
        lastBufferedExtendoPosition = null;
        lastBufferedBotPose = null;

        // init filter
        this.filterLength = filterLength;
        samplePoses = new ArrayList<>();

        this.targetingMethod = targetingMethod;
    }

    public Pose2d getLastBufferedBotPose() {
        return lastBufferedBotPose;
    }

    /**
     * Initializes with timeBuffer=0.045, and filterLength=5.
     */
    public SampleDataBufferFilter(LinearSlidesSubsystem linearSlides, LocalizationSubsystem localization, SampleTargetingMethod targetingMethod) {
        this(linearSlides, localization, 0.03, 3, targetingMethod);
    }

    /**
     * Updates the filter with most recent sample data
     * @param closestSamplePosition
     */
    public void updateFilterData(ArrayList<double[]> sampleTranslations, ArrayList<Double> sampleAngles, double[] closestSamplePosition) {
        if (closestSamplePosition==null || lastBufferedExtendoPosition==null || lastBufferedBotPose==null) return;

        Pose2d sampleToAppend;

        if (samplePoses.isEmpty()) {
            sampleToAppend = getSampleFieldPosition(closestSamplePosition);

            // TODO: add trnaslation case
            double extendoTargetDistance = 0;
            if (targetingMethod==SampleTargetingMethod.ROTATION) {
                extendoTargetDistance = getRotationTargetExtendoDistance(sampleToAppend);
            }
            if (extendoTargetDistance > ExtendoConstants.MAX_EXTENSION) {
                sampleToAppend = null;
            }
        }
        // get sample closest to current samples in filter
        else {
            double closestSampleDistance = Double.MAX_VALUE;
            Pose2d closestSamplePose = null;

            for (int i = 0; i < Math.min(sampleTranslations.size(), sampleAngles.size()); i++) {
                double[] sampleCameraPosition = new double[]{
                        sampleTranslations.get(i)[0],
                        sampleTranslations.get(i)[1],
                        sampleAngles.get(i)
                };

                // calculate sample position
                Pose2d sampleFieldPosition = getSampleFieldPosition(sampleCameraPosition);

                // TODO: add trnaslation case
                double extendoTargetDistance = 0;
                if (targetingMethod==SampleTargetingMethod.ROTATION) {
                    extendoTargetDistance = getRotationTargetExtendoDistance(sampleFieldPosition);
                }
                if (extendoTargetDistance > ExtendoConstants.MAX_EXTENSION) {
                    continue;
                }

                double dx = samplePoses.get(0).getX() - sampleFieldPosition.getX();
                double dy = samplePoses.get(0).getY() - sampleFieldPosition.getY();
                double distance = Math.hypot(dx, dy);

                if (distance < closestSampleDistance) {
                    closestSamplePose = sampleFieldPosition;
                    closestSampleDistance = distance;
                }
            }
            sampleToAppend = closestSamplePose;

            // Cannot locate the sample of the position in the samplePoses list, so just clear it
            if (closestSampleDistance > FILTER_ERROR_TOLERANCE) {
                samplePoses.clear();
            }
        }

        // Sample being null means it is outside of the extendo's reach
        if (sampleToAppend==null) return;

        //// Append current data to end of filter
        samplePoses.add(sampleToAppend);


        //// Remove data outside filter
        while (samplePoses.size()>filterLength) samplePoses.remove(0);

    }
    
    private Pose2d getSampleFieldPosition(double[] samplePosition) {
        // calculate sample position
        ExtendoState extendoPosition = lastBufferedExtendoPosition;
        Pose2d botPose = lastBufferedBotPose;

        // unpack bot pose & extendo state
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();
        double x_extendo = extendoPosition.getLength();

        // get cv readings
        double x_sample_cam = samplePosition[0];
        double y_sample_cam = samplePosition[1];
        double theta_sample_cam = samplePosition[2];

        // convert to robot frame
        double x_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[0] + x_extendo + x_sample_cam;
        double y_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[1] + y_sample_cam;
        double theta_sample_bot = Math.atan2(y_sample_bot, x_sample_bot);
        double distance_sample_bot = Math.hypot(x_sample_bot, y_sample_bot);
        double bearing = heading_bot + theta_sample_bot;

        // find real sample coordinates
        double x_sample = x_bot + distance_sample_bot * Math.cos(bearing);
        double y_sample = y_bot + distance_sample_bot * Math.sin(bearing);
        double theta_sample = (theta_sample_cam + heading_bot) - Math.PI / 2;
        
        return new Pose2d(
                x_sample,
                y_sample,
                new Rotation2d(theta_sample)
        );
    }

    public void clearFilterData() {
        samplePoses.clear();
    }

    /**
     * @return if samplePoses.size()==filterLength.
     */
    public boolean isFilterFull() {
        return samplePoses.size()==filterLength;
    }

    /**
     * @return samplePoses.size()
     */
    public int getFilterLength() {
        return samplePoses.size();
    }

    /**
     * @return the average sample position (relative to the field).
     */
    public Pose2d getFilteredSamplePosition(Telemetry telemetry) {
        if (!isFilterFull()) return null;

        ArrayList<Double> x_sample_sorted = new ArrayList<>();
        ArrayList<Double> y_sample_sorted = new ArrayList<>();
        ArrayList<Double> theta_sample_list = new ArrayList<>();
        double[] theta_sample_vector_sum = new double[]{0,0};
        for (Pose2d samplePose : samplePoses) {
            x_sample_sorted.add(samplePose.getX());
            y_sample_sorted.add(samplePose.getY());
            theta_sample_list.add(samplePose.getHeading());
            theta_sample_vector_sum[0] += Math.cos(samplePose.getHeading());
            theta_sample_vector_sum[1] += Math.sin(samplePose.getHeading());
        }
        Collections.sort(x_sample_sorted);
        Collections.sort(y_sample_sorted);

        telemetry.addData("x_sample_sorted", x_sample_sorted);
        telemetry.addData("y_sample_sorted", y_sample_sorted);
        telemetry.addData("theta_sample_list", theta_sample_list);
        telemetry.update();

        // Use median for x and y
        // Use mean for angle because median would be very difficult due to wrapping
        double x_sample;
        double y_sample;
        double theta_sample;

        // get x_sample, y_sample
        double x1 = x_sample_sorted.get(filterLength/2);
        double y1 = y_sample_sorted.get(filterLength/2);
        if (filterLength%2 == 0) {
            double x2 = x_sample_sorted.get(filterLength/2 - 1);
            double y2 = y_sample_sorted.get(filterLength/2 - 1);
            x_sample = (x1+x2)/2.0;
            y_sample = (y1+y2)/2.0;
        } else {
            x_sample = x1;
            y_sample = y1;
        }

        // get theta_sample
        if (Math.hypot(theta_sample_vector_sum[0],theta_sample_vector_sum[1])==0) {
            theta_sample = samplePoses.get(0).getHeading();
        } else {
            theta_sample = Math.atan2(theta_sample_vector_sum[1],theta_sample_vector_sum[0]);
        }

        return new Pose2d(x_sample, y_sample, new Rotation2d(theta_sample));
    }



    /**
     * Add the most recent measurements to the buffer and clear old measurements.
     */
    public void updateBufferData() {
        double currentTime = runtime.seconds();

        // extendo
        if (linearSlides!=null) {
            double currentExtendoPosition = linearSlides.getExtendoPosition();
            bufferedExtendoPositions.add(new double[]{
                    currentExtendoPosition,
                    currentTime
            });
        }

        // botpose
        if (localization!=null) {
            Pose2d currentBotPose = localization.getPose();
            bufferedBotPoses.add(
                    currentBotPose
            );
            bufferedBotPoseTimes.add(
                    currentTime
            );
        }

        //// UPDATE EXTENDO POSITIONS
        // clear data outside buffer
        while (!bufferedExtendoPositions.isEmpty() && currentTime - bufferedExtendoPositions.get(0)[1] > timeBuffer) {
            bufferedExtendoPositions.remove(0);
        }

        // store latest data
        if (!bufferedExtendoPositions.isEmpty()) {
            lastBufferedExtendoPosition = new ExtendoState(bufferedExtendoPositions.get(0)[0]);
        }

        //// UPDATE BOT POSES
        // clear data outside buffer
        while (!bufferedBotPoses.isEmpty() && currentTime - bufferedBotPoseTimes.get(0) > timeBuffer) {
            bufferedBotPoses.remove(0);
            bufferedBotPoseTimes.remove(0);
        }

        // store latest data
        if (!bufferedBotPoses.isEmpty()) {
            lastBufferedBotPose = bufferedBotPoses.get(0);
        }
    }


    /**
     * Gets the target position of extendo if the robot rotates to reach the given sample
     * @param samplePose
     * @return
     */
    private double getRotationTargetExtendoDistance(Pose2d samplePose) {
        // Unpack bot pose
        Pose2d botPose = localization.getPose();
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();

        // Unpack sample pose
        double x_sample = samplePose.getX();
        double y_sample = samplePose.getY();

        // Convert global sample pose to robot frame
        double dx = x_sample - x_bot;
        double dy = y_sample - y_bot;
        double sin = Math.sin(-heading_bot);
        double cos = Math.cos(-heading_bot);
        double x_sample_bot = dx*cos - dy*sin;
        double y_sample_bot = dx*sin + dy*cos;

        // Calculate heading difference given horizontal claw offset
        double r_sample_bot_norm = Math.hypot(x_sample_bot, y_sample_bot);
        double theta_sample_tangent = Math.atan2(y_sample_bot, x_sample_bot) - Math.acos(OverheadCameraSubsystem.CAMERA_OFFSET[1]/r_sample_bot_norm);

        // Extendo prep calculations
        double rcos = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.cos(theta_sample_tangent);
        double rsin = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.sin(theta_sample_tangent);
        double d_sample_bot = Math.hypot(rcos-x_sample_bot, rsin-y_sample_bot);

        ExtendoState extendoTarget = new ExtendoState(
                d_sample_bot - (OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0])
        );

        return extendoTarget.getLength();
    }

}
