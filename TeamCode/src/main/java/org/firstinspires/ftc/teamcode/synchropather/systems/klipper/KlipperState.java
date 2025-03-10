package org.firstinspires.ftc.teamcode.synchropather.systems.klipper;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a klipper position.
 */
public class KlipperState extends RobotState {
    private final double servoPosition;

    public KlipperState(double servoPosition) {
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
    public KlipperState plus(KlipperState addend) {
        return new KlipperState(servoPosition + addend.getPosition());
    }
    public KlipperState minus(KlipperState addend) {
        return new KlipperState(servoPosition - addend.getPosition());
    }
    public KlipperState times(double factor) {
        return new KlipperState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "KlipperState";
    }
}
