package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class DepositSubsystem extends SubInDepSubsystem<
        DepositSubsystem.State,
        DepositSubsystem.SemidirectControlData,
        DepositSubsystem.DirectControlData
> {
    private HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public DepositSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    public enum State {
        TRANSFER_O(0, 0, 0),
        TRANSFER_C(0, 0, 0),
        APPROACH_C(0, 0, 0),
        DEPOSIT(0, 0, 0),
        REST(0, 0, 0);

        private PositionTargetData data;

        State(double depositWristPos, double depositClawPos, double depositSlidePos){
            data.depositWristPos = depositWristPos;
            data.depositClawPos = depositClawPos;
            data.depositLeftSlidePos = depositSlidePos;
            data.depositRightSlidePos = depositSlidePos;
        }
    }

    public static class PositionTargetData {
        private double
                depositWristPos,
                depositClawPos,
                depositLeftSlidePos,
                depositRightSlidePos;
    }

    public static class DirectControlData {
        public double
                depositWristPower,
                depositClawPower,
                depositLeftSlidePower,
                depositRightSlidePower;
    }

    public static class SemidirectControlData {
        public double
                depositWristPower,
                depositClawPower,
                depositSlidePower;
    }

    @Override
    public void semidirectControl(SemidirectControlData data) {
        targetData.depositClawPos = InDepSubsystem.clamp(
                targetData.depositClawPos + data.depositClawPower, 0, 1
        );
        targetData.depositWristPos = InDepSubsystem.clamp(
                targetData.depositWristPos + data.depositWristPower, 0, 1
        );
        targetData.depositLeftSlidePos = InDepSubsystem.clamp(
                targetData.depositLeftSlidePos
                        + 0.5*(InDepSubsystem.sigmoid(20*data.depositSlidePower-10)+0.5),
                0, 1
        );
        targetData.depositRightSlidePos = InDepSubsystem.clamp(
                targetData.depositRightSlidePos
                        + 0.5*(InDepSubsystem.sigmoid(20*data.depositSlidePower-10)+0.5),
                0, 1
        );
    }

    @Override
    public void directControl(DirectControlData data) {
        targetData.depositClawPos += data.depositClawPower;
        targetData.depositWristPos += data.depositWristPower;
        targetData.depositLeftSlidePos += data.depositLeftSlidePower;
        targetData.depositRightSlidePos += data.depositRightSlidePower;
    }

    public void setState(State state) {
        this.state = state;
        this.targetData = state.data;

        robot.depositClaw.setPosition(targetData.depositClawPos);
        robot.depositWrist.setPosition(targetData.depositWristPos);
        robot.leftDepositLift.setPosition(targetData.depositLeftSlidePos);
        robot.rightDepositLift.setPosition(targetData.depositRightSlidePos);
    }

    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
                approxEq(robot.depositClaw.getPosition(), targetData.depositClawPos) &&
                approxEq(robot.depositWrist.getPosition(), targetData.depositWristPos) &&
                approxEq(robot.leftDepositLift.getPosition(), targetData.depositLeftSlidePos) &&
                approxEq(robot.rightDepositLift.getPosition(), targetData.depositRightSlidePos)
        );
    }

}
