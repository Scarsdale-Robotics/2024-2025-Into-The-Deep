package org.firstinspires.ftc.teamcode.synchropather.systems.vArm;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by vertical arm Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class VArmConstants {

    // Left servo arm homing positions
    public static double armLeftUpPosition = 0.8;
    public static double armLeftTransferPosition = 0.94;

//    // Right servo arm homing positions
//    public static double armRightClipperPosition = 0.65;
//    public static double armRightTransferPosition = 0.79;

    /**
     * How much is needed to be added onto the left arm's position to get the right arm's position.
     */
    public static double SERVO_DIFFERENCE = -0.15;

    /**
     *  Max elbow velocity in servo/sec.
     */
    public static double MAX_VELOCITY = positionToScaled(4);

    /**
     *  Max elbow acceleration in servo/sec^2.
     */
    public static double MAX_ACCELERATION = positionToScaled(4);

    private static double positionToScaled(double position) {
        return position / Math.abs(armLeftUpPosition - armLeftTransferPosition);
    }

}
