package org.firstinspires.ftc.teamcode.synchropather.systems.elbow;

import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem.ElbowPosition;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;


public class ElbowState extends RobotState {

    private final ElbowPosition position;

    public ElbowState(ElbowPosition position) {
        this.position = position;
    }
    public ElbowPosition getPosition() {
        return position;
    }

    @Override
    public String toString() {
        return position.NAME;
    }

    @Override
    public String getDisplayName() {
        return "Elbow";
    }
}
