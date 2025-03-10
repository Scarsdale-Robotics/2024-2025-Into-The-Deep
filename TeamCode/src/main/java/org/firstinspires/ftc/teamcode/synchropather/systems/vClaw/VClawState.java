package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a vertical claw position.
 */
public class VClawState extends RobotState {
    private final double servoPosition;

    public VClawState(double servoPosition) {
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
    public VClawState plus(VClawState addend) {
        return new VClawState(servoPosition + addend.getPosition());
    }
    public VClawState minus(VClawState addend) {
        return new VClawState(servoPosition - addend.getPosition());
    }
    public VClawState times(double factor) {
        return new VClawState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "VClawState";
    }
}
