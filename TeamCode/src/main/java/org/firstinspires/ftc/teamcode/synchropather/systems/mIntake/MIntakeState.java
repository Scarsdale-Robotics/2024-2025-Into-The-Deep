package org.firstinspires.ftc.teamcode.synchropather.systems.mIntake;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Contains a magazine intake position.
 */
public class MIntakeState extends RobotState {
    private final double servoPosition;

    public MIntakeState(double servoPosition) {
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
    public MIntakeState plus(MIntakeState addend) {
        return new MIntakeState(servoPosition + addend.getPosition());
    }
    public MIntakeState minus(MIntakeState addend) {
        return new MIntakeState(servoPosition - addend.getPosition());
    }
    public MIntakeState times(double factor) {
        return new MIntakeState(servoPosition * factor);
    }
    @Override
    public String toString() {
        return String.format("%s servo position", servoPosition);
    }
    @Override
    public String getDisplayName() {
        return "MIntakeState";
    }
}
