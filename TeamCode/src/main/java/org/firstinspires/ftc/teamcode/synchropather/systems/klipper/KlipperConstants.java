package org.firstinspires.ftc.teamcode.synchropather.systems.klipper;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by magazine intake Movements to reference important servo positions.
 */
@Config
public class KlipperConstants {

    // Klipper end positions
    public static double closedPosition = 0.62;
    public static double openPosition = 0;

    //    public static double MAX_VELOCITY = 0.5;
    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 2;

    public static double MOVEMENT_DURATION = Math.abs(closedPosition-openPosition)/MAX_VELOCITY;

}
