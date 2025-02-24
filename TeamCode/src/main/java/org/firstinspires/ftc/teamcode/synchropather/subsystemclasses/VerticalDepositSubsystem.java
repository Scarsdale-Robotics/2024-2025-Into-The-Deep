package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;

public class VerticalDepositSubsystem {

    private final Servo leftVerticalArm;
    private final Servo rightVerticalArm;
    private final Servo verticalClaw;

    private final double armLeftZeroPosition = VArmConstants.armLeftZeroPosition;
    private final double armLeftPiPosition = VArmConstants.armLeftPiPosition;
    private final double armRightZeroPosition = VArmConstants.armRightZeroPosition;
    private final double armRightPiPosition = VArmConstants.armRightPiPosition;
    private final double clawGrabPosition = VClawConstants.GRAB_POSITION;
    private final double clawReleasePosition = VClawConstants.RELEASE_POSITION;

    public VerticalDepositSubsystem(
            Servo leftVerticalArm,
            Servo rightVerticalArm,
            Servo verticalClaw
    ) {
        this.leftVerticalArm = leftVerticalArm;
        this.rightVerticalArm = rightVerticalArm;
        this.verticalClaw = verticalClaw;
    }

    /**
     * Claw action
     */
    public void grab() {
        verticalClaw.setPosition(clawGrabPosition);
    }

    /**
     * Claw action
     */
    public void release() {
        verticalClaw.setPosition(clawReleasePosition);
    }

    /**
     * Raw servo access -- USE CAREFULLY
     * @param clawPosition between [0, 1]
     */
    public void setClawPosition(double clawPosition) {
        verticalClaw.setPosition(clawPosition);
    }

    /**
//     * 0 is back into the robot (never use)
//     * 1 is pickup position
     * @param armPosition between [0, 1]
     */
    public void setArmPosition(double armPosition) {
        double[] armServoPositions = toArmServoPositions(armPosition);
        leftVerticalArm.setPosition(armServoPositions[0]);
        rightVerticalArm.setPosition(armServoPositions[1]);
    }

    /**
     * @param verticalArmPosition between [0, 1]
     * @return {leftVerticalArmPosition, rightVerticalArmPosition}
     */
    private double[] toArmServoPositions(double verticalArmPosition) {
        double leftVerticalArmPosition = armLeftZeroPosition + verticalArmPosition*(armLeftPiPosition - armLeftZeroPosition);
        double rightVerticalArmPosition = armRightZeroPosition + verticalArmPosition*(armRightPiPosition - armRightZeroPosition);
        return new double[]{leftVerticalArmPosition, rightVerticalArmPosition};
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians >= Math.PI) radians -= 2*Math.PI;
        while (radians < -Math.PI) radians += 2*Math.PI;
        return radians;
    }

}
