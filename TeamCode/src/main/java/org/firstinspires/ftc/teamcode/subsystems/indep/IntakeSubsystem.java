package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class IntakeSubsystem extends SubInDepSubsystem<IntakeSubsystem.State> {

    private HardwareRobot robot;
    private State state;

    public IntakeSubsystem(HardwareRobot robot) {
        this.robot = robot;
        this.state = State.REST;
    }

    public enum State {
        APPROACH_O(0, 0, 0, 0),
        INTAKE_O(0, 0, 0, 0),
        INTAKE_C(0, 0, 0, 0),
        APPROACH_C(0, 0, 0, 0),
        TRANSFER_C(0, 0, 0, 0),
        TRANSFER_O(0, 0, 0, 0),
        REST(0, 0, 0, 0);

        private final double intakePivotPos, intakeWristPos, intakeClawPos, intakeSlidePos;

        State(double intakePivotPos, double intakeWristPos, double intakeClawPos, double intakeSlidePos){
            this.intakePivotPos = intakePivotPos;
            this.intakeWristPos = intakeWristPos;
            this.intakeClawPos = intakeClawPos;
            this.intakeSlidePos = intakeSlidePos;
        }
    }

    public void setState(State state) {
        this.state = state;
        robot.intakeClaw.setPosition(state.intakeClawPos);
        robot.intakeWrist.setPosition(state.intakeWristPos);
        robot.intakePivot.setPosition(state.intakePivotPos);
        robot.leftIntakeLift.setPosition(state.intakeSlidePos);
        robot.rightIntakeLift.setPosition(state.intakeSlidePos);
    }

    public State getState() {
        return state;
    }

    public boolean jobFulfilled() {
        return (
            approxEq(robot.intakeClaw.getPosition(), state.intakeClawPos) &&
            approxEq(robot.intakeWrist.getPosition(), state.intakeWristPos) &&
            approxEq(robot.intakePivot.getPosition(), state.intakePivotPos) &&
            approxEq(robot.leftIntakeLift.getPosition(), state.intakeSlidePos) &&
            approxEq(robot.rightIntakeLift.getPosition(), state.intakeSlidePos)
        );
    }

}
