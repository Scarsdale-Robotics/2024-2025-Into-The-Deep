package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LimelightConstants {

    // Sample pose estimation coefficients
    public static double theta_incline = 0; // radians
    public static double k1 = 480;
    public static double k2 = 640;
    public static double cz = 3.625; // inches from above the field
    public static double dist_covariance = 0.25;


    // Sample pose estimation probability function
    public static double FIELD_RESOLUTION = 1; // inches
    public static double DECAY_TIME = 0.5;
    public static final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);

    public static double SAMPLE_PROBABILITY_THRESHOLD = 3;
    public static double SAMPLE_CLUSTER_SIZE_THRESHOLD = 0.5; // inches^2

}
