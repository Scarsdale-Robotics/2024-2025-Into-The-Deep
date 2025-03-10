package org.firstinspires.ftc.teamcode.synchropather.systems.vArm;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a vertical arm position.
 */
public class VArmState extends RobotState {
    private final double leftArmPosition;

    public VArmState(double leftArmPosition) {
        this.leftArmPosition = leftArmPosition;
    }
    public double getPosition() {
        return leftArmPosition;
    }
    public double sign() {
        return Math.signum(leftArmPosition);
    }
    public double abs() {
        return Math.abs(leftArmPosition);
    }
    public VArmState plus(VArmState addend) {
        return new VArmState(leftArmPosition + addend.getPosition());
    }
    public VArmState minus(VArmState addend) {
        return new VArmState(leftArmPosition - addend.getPosition());
    }
    public VArmState times(double factor) {
        return new VArmState(leftArmPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", leftArmPosition);
    }
    @Override
    public String getDisplayName() {
        return "VArmState";
    }
}
