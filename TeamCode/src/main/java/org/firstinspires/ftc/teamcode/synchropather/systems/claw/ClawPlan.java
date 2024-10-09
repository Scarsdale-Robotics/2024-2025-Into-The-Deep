package org.firstinspires.ftc.teamcode.synchropather.systems.claw;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;

import java.util.ArrayList;

public class ClawPlan extends Plan<ClawState> {


    private RobotSystem robot;
    public ClawPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.CLAW, movements);
        this.robot = robot;
    }

    public void loop() {
        // Desired states
        ClawState desiredState = getCurrentState();


        robot.inDep.setClawPosition(desiredState.getPosition());
    }
}
