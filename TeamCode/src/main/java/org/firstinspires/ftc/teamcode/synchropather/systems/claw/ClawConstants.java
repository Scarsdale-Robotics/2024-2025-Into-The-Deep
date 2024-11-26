package org.firstinspires.ftc.teamcode.synchropather.systems.claw;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Claw Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class ClawConstants {

    /**
     *  Max claw velocity in servovalue/s.
     */
    public static double MAX_VELOCITY = 4;
    /**
     *  Max claw acceleration in servovalue/s^2.
     */
    public static double MAX_ACCELERATION = 8;

    public static double OPEN_POSITION = 0.3;
    public static double CLOSED_POSITION = 0.19;

}
