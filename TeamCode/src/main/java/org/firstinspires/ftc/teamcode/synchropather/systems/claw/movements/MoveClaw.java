package org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;

public class MoveClaw extends Movement {

    private TimeSpan timeSpan;
    private ClawState startState, endState;

    public MoveClaw(double startTime, ClawState startState, ClawState endState) {
        super(new TimeSpan(startTime, startTime), MovementType.ELBOW);
        this.startState = startState;
        this.endState = endState;
    }

    @Override
    public double getMinDuration() {
        return 0;
    }

    @Override
    public RobotState getState(double elapsedTime) {
        return endState;
    }

    @Override
    public RobotState getVelocity(double elapsedTime) {
        return null;
    }

    @Override
    public RobotState getAcceleration(double elapsedTime) {
        return null;
    }

    @Override
    public RobotState getStartState() {
        return startState;
    }

    @Override
    public RobotState getEndState() {
        return endState;
    }

    @Override
    public String getDisplayName() {
        return "MoveClaw";
    }
}
