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

    private final double SERVO_DIFFERENCE = VArmConstants.SERVO_DIFFERENCE;
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
     * Sets the left arm position to the given value and the right arm position to
     * leftArmPosition + SERVO_DIFFERENCE.
     * @param leftArmPosition between [0, 1]
     */
    public void setArmPosition(double leftArmPosition) {
        leftVerticalArm.setPosition(leftArmPosition);
        rightVerticalArm.setPosition(leftArmPosition+VArmConstants.SERVO_DIFFERENCE);
    }

}
