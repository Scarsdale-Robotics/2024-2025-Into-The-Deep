package org.firstinspires.ftc.teamcode.synchropather.systems.mLoader;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a magazine loader position.
 */
public class MLoaderState extends RobotState {
    private final double servoPosition;

    public MLoaderState(double servoPosition) {
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
    public MLoaderState plus(MLoaderState addend) {
        return new MLoaderState(servoPosition + addend.getPosition());
    }
    public MLoaderState minus(MLoaderState addend) {
        return new MLoaderState(servoPosition - addend.getPosition());
    }
    public MLoaderState times(double factor) {
        return new MLoaderState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "MLoaderState";
    }
}
