package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;

public class LiftState extends RobotState {
    public static final LiftState zero = new LiftState(0);
    private final double height;
    private final double max_height = 999;

    public LiftState(double height) {
        this.height = height;
    }
    public double getHeight() {
        return height;
    }
    public double sign() {
        return Math.signum(height);
    }
    public double abs() {
        return Math.abs(height);
    }
    public LiftState plus(LiftState addend) {
        return new LiftState(height + addend.getHeight());
    }
    public LiftState minus(LiftState addend) {
        return new LiftState(height - addend.getHeight());
    }
    public LiftState times(double factor) {
        return new LiftState(height * factor);
    }
    @Override
    public String toString() {
        return String.format("%s ticks", height);
    }
    @Override
    public String getDisplayName() {
        return "Lift";
    }
}
