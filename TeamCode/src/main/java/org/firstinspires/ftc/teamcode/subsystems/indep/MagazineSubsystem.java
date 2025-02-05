package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class MagazineSubsystem extends SubInDepSubsystem<
        MagazineSubsystem.State,
        MagazineSubsystem.SemidirectControlData,
        MagazineSubsystem.DirectControlData
> {

    private HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public MagazineSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    @Override
    public void semidirectControl(SemidirectControlData data) {
        targetData.magServoPos = InDepSubsystem.clamp_0p1(
                targetData.magServoPos + data.magServoPower
        );
    }

    @Override
    public void directControl(DirectControlData data) {
        targetData.magServoPos += data.magServoPower;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        this.targetData = state.data;

        robot.clipPusher.setPosition(targetData.magServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.clipPusher.getPosition(), targetData.magServoPos)
        );
    }

    public enum State {
        DEQUEUE(0),
        REST(0);

        private PositionTargetData data;

        State(double magServoPos){
            data.magServoPos = magServoPos;
        }
    }

    public static class PositionTargetData {
        public double magServoPos;
    }

    public static class DirectControlData {
        public double magServoPower;
    }

    public static class SemidirectControlData {
        public double magServoPower;
    }

}
