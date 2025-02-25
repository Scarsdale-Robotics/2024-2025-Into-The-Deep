package org.firstinspires.ftc.teamcode.synchropather.systems.mIntake;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by magazine intake Movements to reference important servo positions.
 */
@Config
public class MIntakeConstants {

    // Magazine intake end positions
    public static double openPosition = 0.5;
    public static double closedPosition = 0.21;
    public static double upPosition = 0.7;

    //    public static double MAX_VELOCITY = 0.5;
    public static double MAX_VELOCITY = 4;
    public static double MAX_ACCELERATION = 2;

}
