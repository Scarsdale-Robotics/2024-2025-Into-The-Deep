package org.firstinspires.ftc.teamcode.synchropather.systems.elbow;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class ElbowState extends RobotState {
    public static final ElbowState zero = new ElbowState(0);
    private double servoPosition = 0;

    public ElbowState(double servoPosition) {
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
    public ElbowState plus(ElbowState addend) {
        return new ElbowState(servoPosition + addend.getPosition());
    }
    public ElbowState minus(ElbowState addend) {
        return new ElbowState(servoPosition - addend.getPosition());
    }
    public ElbowState times(double factor) {
        return new ElbowState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s ticks", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "Elbow";
    }
}
