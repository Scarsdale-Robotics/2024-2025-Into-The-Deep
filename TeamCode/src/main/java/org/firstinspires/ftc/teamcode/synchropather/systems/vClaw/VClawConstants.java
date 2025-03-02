package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by vertical claw Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class VClawConstants {

    // Claw positions
    public static double GRAB_POSITION = 0.72;
    public static double LOOSELY_GRABBED_POSITION = 0.78;
    public static double RELEASE_POSITION = 1;

    /**
     * in servo/sec
     */
    public static double MAX_SPEED = 4;

    public static double MOVEMENT_TIME = Math.abs(GRAB_POSITION - RELEASE_POSITION) / MAX_SPEED;

}
