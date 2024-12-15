package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class MagazineSubsystem extends SubInDepSubsystem<MagazineSubsystem.State> {

    private HardwareRobot robot;
    private State state;

    public MagazineSubsystem(HardwareRobot robot) {
        this.robot = robot;
        this.state = State.REST;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        robot.clipPusher.setPosition(state.magServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.clipPusher.getPosition(), state.magServoPos)
        );
    }

    public enum State {
        DEQUEUE(0),
        REST(0);

        private final double magServoPos;

        State(double magServoPos){
            this.magServoPos = magServoPos;
        }
    }

}
