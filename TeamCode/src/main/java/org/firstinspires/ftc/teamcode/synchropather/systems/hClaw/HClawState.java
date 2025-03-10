package org.firstinspires.ftc.teamcode.synchropather.systems.hClaw;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a horizontal claw position.
 */
public class HClawState extends RobotState {
    private final double servoPosition;

    public HClawState(double servoPosition) {
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
    public HClawState plus(HClawState addend) {
        return new HClawState(servoPosition + addend.getPosition());
    }
    public HClawState minus(HClawState addend) {
        return new HClawState(servoPosition - addend.getPosition());
    }
    public HClawState times(double factor) {
        return new HClawState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "HClawState";
    }
}
