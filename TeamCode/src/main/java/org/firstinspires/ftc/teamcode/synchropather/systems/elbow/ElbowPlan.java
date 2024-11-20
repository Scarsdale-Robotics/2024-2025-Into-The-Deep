package org.firstinspires.ftc.teamcode.synchropather.systems.elbow;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;

import java.util.ArrayList;

public class ElbowPlan extends Plan<ElbowState> {


    private RobotSystem robot;
    public ElbowPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.robot = robot;
    }

    public void loop() {
        // Desired states
        ElbowState desiredState = getCurrentState();


        robot.inDep.setElbowPosition(desiredState.getPosition());
    }

    @Override
    public void stop() {

    }
}
