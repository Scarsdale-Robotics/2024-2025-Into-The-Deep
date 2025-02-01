package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristConstants;

public class HorizontalIntakeSubsystem {

    private final Servo leftHorizontalArm;
    private final Servo rightHorizontalArm;
    private final Servo horizontalWrist;
    private final Servo horizontalClaw;

    private final double armLeftZeroPosition = HArmConstants.armLeftZeroPosition;
    private final double armLeftPiPosition = HArmConstants.armLeftPiPosition;
    private final double armRightZeroPosition = HArmConstants.armRightZeroPosition;
    private final double armRightPiPosition = HArmConstants.armRightPiPosition;
    private final double wristMinusNinetyPosition = HWristConstants.wristMinusNinetyPosition;
    private final double wristPlusNinetyPosition = HWristConstants.wristPlusNinetyPosition;
    private final double clawGrabPosition = HClawConstants.GRAB_POSITION;
    private final double clawReleasePosition = HClawConstants.RELEASE_POSITION;

    public HorizontalIntakeSubsystem(
            Servo leftHorizontalArm,
            Servo rightHorizontalArm,
            Servo horizontalWrist,
            Servo horizontalClaw
    ) {
        this.leftHorizontalArm = leftHorizontalArm;
        this.rightHorizontalArm = rightHorizontalArm;
        this.horizontalWrist = horizontalWrist;
        this.horizontalClaw = horizontalClaw;
    }

    /**
     * Claw action
     */
    public void grab() {
        horizontalClaw.setPosition(clawGrabPosition);
    }

    /**
     * Claw action
     */
    public void release() {
        horizontalClaw.setPosition(clawReleasePosition);
    }

    /**
     * Raw servo access -- USE CAREFULLY
     * @param clawPosition between [0, 1]
     */
    public void setClawPosition(double clawPosition) {
        horizontalClaw.setPosition(clawPosition);
    }

    /**
     * @param radians normalized to (-pi,pi]
     */
    public void setWristAngle(double radians) {
        double sampleAngleProportion = (normalizeAngle(radians) - -Math.PI/2) / Math.PI;
        double servoPosition = wristMinusNinetyPosition + sampleAngleProportion*(wristPlusNinetyPosition - wristMinusNinetyPosition);
        horizontalWrist.setPosition(servoPosition);
    }

    /**
     * 0 is back into the robot (never use)
     * 1 is pickup position
     * @param armPosition between [0, 1]
     */
    public void setArmPosition(double armPosition) {
        double[] armServoPositions = toArmServoPositions(armPosition);
        leftHorizontalArm.setPosition(armServoPositions[0]);
        rightHorizontalArm.setPosition(armServoPositions[1]);
    }

    /**
     * @param horizontalArmPosition between [0, 1] where 0 is at spec maker and 1 is on the ground
     * @return {leftHorizontalArmPosition, rightHorizontalArmPosition}
     */
    private double[] toArmServoPositions(double horizontalArmPosition) {
        double leftHorizontalArmPosition = armLeftZeroPosition + horizontalArmPosition*(armLeftPiPosition - armLeftZeroPosition);
        double rightHorizontalArmPosition = armRightZeroPosition + horizontalArmPosition*(armRightPiPosition - armRightZeroPosition);
        return new double[]{leftHorizontalArmPosition, rightHorizontalArmPosition};
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
