package org.firstinspires.ftc.teamcode.synchropather.systems.claw;

import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem.ClawPosition;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;


public class ClawState extends RobotState {

    private final ClawPosition position;

    public ClawState(ClawPosition position) {
        this.position = position;
    }
    public ClawPosition getPosition() {
        return position;
    }

    @Override
    public String toString() {
        return position.NAME;
    }

    @Override
    public String getDisplayName() {
        return "Claw";
    }
}
