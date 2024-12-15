package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class MakerSubsystem extends SubInDepSubsystem<MakerSubsystem.State> {

    private HardwareRobot robot;
    private State state;

    public MakerSubsystem(HardwareRobot robot) {
        this.robot = robot;
        this.state = State.REST;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        robot.maker.setPosition(state.makerServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.maker.getPosition(), state.makerServoPos)
        );
    }

    public enum State {
        UNITE(0),
        REST(0);

        private final double makerServoPos;

        State(double makerServoPos){
            this.makerServoPos = makerServoPos;
        }
    }

}
