package org.firstinspires.ftc.teamcode.synchropather.systems.vArm;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by vertical arm Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class VArmConstants {

    // Left servo arm homing positions
    public static double armLeftClipperPosition = 0.95;
    public static double armLeftTransferPosition = 0.96;
    public static double armLeftPreDepositPosition = 0.6;
    public static double armLeftDepositPosition = 0.5;

    private static double positionToScaled(double position) {
        return position / Math.abs(armLeftClipperPosition - armLeftTransferPosition);
    }

    /**
     * How much is needed to be added onto the left arm's position to get the right arm's position.
     */
    public static double SERVO_DIFFERENCE = -0.15;

    /**
     *  Max elbow velocity in servo/sec.
     */
    public static double MAX_VELOCITY = positionToScaled(2);

    /**
     *  Max elbow acceleration in servo/sec^2.
     */
    public static double MAX_ACCELERATION = positionToScaled(4);

}
