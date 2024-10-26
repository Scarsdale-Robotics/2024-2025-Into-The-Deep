package org.firstinspires.ftc.teamcode.synchropather.systems.claw;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class ClawState extends RobotState {
    public static final ClawState zero = new ClawState(0);
    private double servoPosition = 0;

    public ClawState(double servoPosition) {
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
    public ClawState plus(ClawState addend) {
        return new ClawState(servoPosition + addend.getPosition());
    }
    public ClawState minus(ClawState addend) {
        return new ClawState(servoPosition - addend.getPosition());
    }
    public ClawState times(double factor) {
        return new ClawState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s ticks", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "Claw";
    }
}
