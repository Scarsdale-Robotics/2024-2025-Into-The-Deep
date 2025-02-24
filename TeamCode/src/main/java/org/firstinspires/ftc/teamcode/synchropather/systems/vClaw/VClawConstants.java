package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by vertical claw Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class VClawConstants {

    // Claw positions
    public static double GRAB_POSITION = 0.5;
    public static double RELEASE_POSITION = 0.22;

    /**
     * in servo/sec
     */
    public static double MAX_SPEED = 2;

    public static double MOVEMENT_TIME = Math.abs(GRAB_POSITION - RELEASE_POSITION) / MAX_SPEED;

}
