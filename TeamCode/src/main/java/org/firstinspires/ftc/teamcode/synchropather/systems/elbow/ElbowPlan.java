package org.firstinspires.ftc.teamcode.synchropather.systems.elbow;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

@Config
public class ElbowPlan extends Plan<ElbowState> {

    private RobotSystem robot;

    public ElbowPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.ELBOW, movements);
        robot.telemetry.addData("[SYNCHROPATHER] ElbowPlan desiredState.getPosition()", 0);
        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        ElbowState desiredState = getCurrentState();

        robot.inDep.setElbowPosition(desiredState.getPosition());

        robot.telemetry.addData("[SYNCHROPATHER] ElbowPlan desiredState.getPosition()", desiredState.getPosition());
        robot.telemetry.update();
    }

    @Override
    public void stop() {}
}
