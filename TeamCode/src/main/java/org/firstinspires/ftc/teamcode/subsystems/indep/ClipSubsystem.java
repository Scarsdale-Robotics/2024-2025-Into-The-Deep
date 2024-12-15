package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class ClipSubsystem extends SubInDepSubsystem<ClipSubsystem.State> {

    private HardwareRobot robot;
    private State state;

    public ClipSubsystem(HardwareRobot robot) {
        this.robot = robot;
        this.state = State.REST;
    }

    @Override
    public void setState(State state) {
        this.state = state;
        robot.clipIntake.setPosition(state.clipIntakeServoPos);
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean jobFulfilled() {
        return (
            approxEq(robot.clipIntake.getPosition(), state.clipIntakeServoPos)
        );
    }

    public enum State {
        OPEN(0),
        REST(0);

        private final double clipIntakeServoPos;

        State(double clipIntakeServoPos){
            this.clipIntakeServoPos = clipIntakeServoPos;
        }
    }

}
