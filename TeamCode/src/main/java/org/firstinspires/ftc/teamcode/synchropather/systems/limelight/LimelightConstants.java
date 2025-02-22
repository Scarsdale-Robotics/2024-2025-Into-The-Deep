package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class LimelightConstants {

    // Sample pose estimation constants
    public static double theta_incline = Math.toRadians(27); // radians
    public static double FOV = 640;
    public static double cz = 7; // inches from above the field

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

    // 156mm by 194.75mm, +15 degrees yaw
    public static Pose2d limelightPosition = new Pose2d(-6.14173,7.66732283,new Rotation2d(Math.toRadians(15)));



}
