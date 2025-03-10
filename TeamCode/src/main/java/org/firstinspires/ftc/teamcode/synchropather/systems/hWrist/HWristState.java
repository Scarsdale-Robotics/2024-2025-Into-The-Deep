package org.firstinspires.ftc.teamcode.synchropather.systems.hWrist;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a horizontal wrist position.
 */
public class HWristState extends RobotState {
    private final double servoPosition;

    public HWristState(double servoPosition) {
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
    public HWristState plus(HWristState addend) {
        return new HWristState(servoPosition + addend.getPosition());
    }
    public HWristState minus(HWristState addend) {
        return new HWristState(servoPosition - addend.getPosition());
    }
    public HWristState times(double factor) {
        return new HWristState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "HWristState";
    }
}
