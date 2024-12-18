package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class LimelightConstants {

    // Sample pose estimation constants
    public static double theta_incline = Math.toRadians(27); // radians
    public static double k1 = -640;
    public static double k2 = 640;
    public static double cz = 7; // inches from above the field
    public static double dist_covariance = 0.25;

    //
    //         ^ +Y (left of robot)
    //         |
    // __________________
    // |                |
    // |                |
    // |      <-\ +H    | --> +X (front of robot)
    // |       _/       |
    // |                |
    // ------------------
    //
    public static Pose2d limelightPosition = new Pose2d(0,0,new Rotation2d(0));


    // Sample pose estimation probability function
    public static double FIELD_RESOLUTION = 1; // inches
    public static double DECAY_TIME = 0.5;
    public static final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);

    public static double SAMPLE_PROBABILITY_THRESHOLD = 3;
    public static double SAMPLE_CLUSTER_SIZE_THRESHOLD = 0.5; // inches^2



}
