package org.firstinspires.ftc.teamcode.synchropather.systems.claw;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

@Config
public class ClawPlan extends Plan<ClawState> {

    private RobotSystem robot;

    public ClawPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.CLAW, movements);
        this.robot = robot;
//        robot.telemetry.addData("[SYNCHROPATHER] ClawPlan desiredState.getPosition()", 0);
//        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        ClawState desiredState = getCurrentState();

        robot.inDep.setClawPosition(desiredState.getPosition());

        robot.telemetry.addData("[SYNCHROPATHER] ClawPlan desiredState.getPosition()", desiredState.getPosition());
        robot.telemetry.update();
    }

    @Override
    public void stop() {}

    public void setRobot(RobotSystem robot) {
        this.robot = robot;
    }
}
