package org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class MFeederState extends RobotState {
    private final double position;

    public MFeederState(double position) {
        this.position = position;
    }
    public double getPosition() {
        return position;
    }
    public double sign() {
        return Math.signum(position);
    }
    public double abs() {
        return Math.abs(position);
    }
    public MFeederState plus(MFeederState addend) {
        return new MFeederState(position + addend.getPosition());
    }
    public MFeederState minus(MFeederState addend) {
        return new MFeederState(position - addend.getPosition());
    }
    public MFeederState times(double factor) {
        return new MFeederState(position * factor);
    }
    @Override
    public String toString() {
        return String.format("%s inches", position);
    }
    @Override
    public String getDisplayName() {
        return "MFeederState";
    }
}
