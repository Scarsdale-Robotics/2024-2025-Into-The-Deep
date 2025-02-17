package org.firstinspires.ftc.teamcode.synchropather.systems.hArm;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a horizontal arm position.
 */
public class HArmState extends RobotState {
    private final double servoPosition;

    public HArmState(double servoPosition) {
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
    public HArmState plus(HArmState addend) {
        return new HArmState(servoPosition + addend.getPosition());
    }
    public HArmState minus(HArmState addend) {
        return new HArmState(servoPosition - addend.getPosition());
    }
    public HArmState times(double factor) {
        return new HArmState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "HArmState";
    }
}
