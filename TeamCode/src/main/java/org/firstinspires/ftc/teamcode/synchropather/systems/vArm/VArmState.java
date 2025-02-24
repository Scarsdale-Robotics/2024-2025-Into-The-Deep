package org.firstinspires.ftc.teamcode.synchropather.systems.vArm;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a vertical arm position.
 */
public class VArmState extends RobotState {
    private final double servoPosition;

    public VArmState(double servoPosition) {
        this.servoPosition = servoPosition;
    }
    public double getPosition() {
        return servoPosition;
    }
    public double sign() {
        return Math.signum(servoPosition);
    }
    public double abs() {
        return Math.abs(servoPosition);
    }
    public VArmState plus(VArmState addend) {
        return new VArmState(servoPosition + addend.getPosition());
    }
    public VArmState minus(VArmState addend) {
        return new VArmState(servoPosition - addend.getPosition());
    }
    public VArmState times(double factor) {
        return new VArmState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "VArmState";
    }
}
