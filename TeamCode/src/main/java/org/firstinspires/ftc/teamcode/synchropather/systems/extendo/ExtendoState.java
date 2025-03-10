package org.firstinspires.ftc.teamcode.synchropather.systems.extendo;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class ExtendoState extends RobotState {
    private final double length;

    public ExtendoState(double length) {
        this.length = length;
    }
    public double getLength() {
        return length;
    }
    public double sign() {
        return Math.signum(length);
    }
    public double abs() {
        return Math.abs(length);
    }
    public ExtendoState plus(ExtendoState addend) {
        return new ExtendoState(length + addend.getLength());
    }
    public ExtendoState minus(ExtendoState addend) {
        return new ExtendoState(length - addend.getLength());
    }
    public ExtendoState times(double factor) {
        return new ExtendoState(length * factor);
    }
    @Override
    public String toString() {
        return String.format("%s inches", length);
    }
    @Override
    public String getDisplayName() {
        return "Extendo";
    }
}
