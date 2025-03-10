package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class MakerSubsystem extends SubInDepSubsystem<
        MakerSubsystem.State,
        MakerSubsystem.SemidirectControlData,
        MakerSubsystem.DirectControlData
> {

    private HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public MakerSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    @Override
    public void semidirectControl(MakerSubsystem.SemidirectControlData data) {
        targetData.makerServoPos = InDepSubsystem.clamp_0p1(
                targetData.makerServoPos + data.makerServoPower
        );
    }

    @Override
    public void directControl(MakerSubsystem.DirectControlData data) {
        targetData.makerServoPos += data.makerServoPower;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        this.targetData = state.data;
        robot.maker.setPosition(targetData.makerServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.maker.getPosition(), targetData.makerServoPos)
        );
    }

    public enum State {
        UNITE(0),
        REST(0);

        private final PositionTargetData data;

        State(double makerServoPos){
            data = new PositionTargetData();
            data.makerServoPos = makerServoPos;
        }
    }

    public static class PositionTargetData {
        public double makerServoPos;
    }

    public static class DirectControlData {
        public double makerServoPower;
    }

    public static class SemidirectControlData {
        public double makerServoPower;
    }

}
