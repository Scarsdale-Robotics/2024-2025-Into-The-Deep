package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class DepositSubsystem {

    private HardwareRobot robot;
    private State state;

    public DepositSubsystem(HardwareRobot robot) {
        this.robot = robot;
        this.state = State.REST;
    }

    public enum State {
        TRANSFER_O(0, 0, 0),
        TRANSFER_C(0, 0, 0),
        APPROACH_C(0, 0, 0),
        DEPOSIT(0, 0, 0),
        REST(0, 0, 0);

        private final double depositWristPos, depositClawPos, depositSlidePos;

        State(double depositWristPos, double depositClawPos, double depositSlidePos){
            this.depositWristPos = depositWristPos;
            this.depositClawPos = depositClawPos;
            this.depositSlidePos = depositSlidePos;
        }
    }

    public void setState(State state) {
        robot.depositClaw.setPosition(state.depositClawPos);
        robot.depositWrist.setPosition(state.depositWristPos);
        robot.leftDepositLift.setPosition(state.depositSlidePos);
        robot.rightDepositLift.setPosition(state.depositSlidePos);
    }

    public State getState() {
        return state;
    }

    private boolean approxEq(double a, double b) {
        double epsilon = 0.000001;
        return Math.abs(a-b) <= epsilon;
    }

    public boolean jobFulfilled() {
        return (
                approxEq(robot.depositClaw.getPosition(), state.depositClawPos) &&
                approxEq(robot.depositWrist.getPosition(), state.depositWristPos) &&
                approxEq(robot.leftDepositLift.getPosition(), state.depositSlidePos) &&
                approxEq(robot.rightDepositLift.getPosition(), state.depositSlidePos)
        );
    }

}
