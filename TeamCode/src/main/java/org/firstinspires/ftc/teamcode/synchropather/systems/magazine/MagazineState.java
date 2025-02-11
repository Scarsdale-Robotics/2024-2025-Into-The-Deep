package org.firstinspires.ftc.teamcode.synchropather.systems.magazine;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class MagazineState extends RobotState {
    private final double length;

    public MagazineState(double length) {
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
    public MagazineState plus(MagazineState addend) {
        return new MagazineState(length + addend.getLength());
    }
    public MagazineState minus(MagazineState addend) {
        return new MagazineState(length - addend.getLength());
    }
    public MagazineState times(double factor) {
        return new MagazineState(length * factor);
    }
    @Override
    public String toString() {
        return String.format("%s inches", length);
    }
    @Override
    public String getDisplayName() {
        return "Magazine";
    }
}
