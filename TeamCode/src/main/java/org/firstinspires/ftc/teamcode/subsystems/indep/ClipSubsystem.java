package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class ClipSubsystem extends SubInDepSubsystem<
        ClipSubsystem.State,
        ClipSubsystem.SemidirectControlData,
        ClipSubsystem.DirectControlData
> {

    private HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public ClipSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    @Override
    public void semidirectControl(ClipSubsystem.SemidirectControlData data) {
        targetData.clipIntakeServoPos = InDepSubsystem.clamp_0p1(
                targetData.clipIntakeServoPos + data.clipIntakeServoPower
        );
    }

    @Override
    public void directControl(ClipSubsystem.DirectControlData data) {
        targetData.clipIntakeServoPos += data.clipIntakeServoPower;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        this.targetData = state.data;
        robot.clipIntake.setPosition(targetData.clipIntakeServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.clipIntake.getPosition(), targetData.clipIntakeServoPos)
        );
    }

    public enum State {
        OPEN(0),
        REST(0);

        private final PositionTargetData data;

        State(double clipIntakeServoPos){
            data = new PositionTargetData();
            data.clipIntakeServoPos = clipIntakeServoPos;
        }
    }

    public static class PositionTargetData {
        public double clipIntakeServoPos;
    }

    public static class DirectControlData {
        public double clipIntakeServoPower;
    }

    public static class SemidirectControlData {
        public double clipIntakeServoPower;
    }

}
