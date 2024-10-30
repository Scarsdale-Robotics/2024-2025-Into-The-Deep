package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;

@Config
@Autonomous(name="Example SynchroPather Claw Picking Up Sample Auto", group = "Calibration")
public class ExampleSynchroPatherPickUpSample extends LinearOpMode {

    RobotSystem robot;
    Synchronizer pickup, dropoff;

    public static double clawOpen = 0.23;
    public static double clawClosed = 0.06;

    public static double elbowUp = 0.275;
    public static double elbowDown = 0.53;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), this);
        robot.inDep.setClawPosition(clawOpen);
        robot.inDep.setElbowPosition(elbowUp);
        initSynchronizers();

        waitForStart();

        while (opModeIsActive()) {
            // Pickup
            while (opModeIsActive() && !gamepad1.square);
            pickup.start();
            while (opModeIsActive() && pickup.update()) {
                robot.localization.update();
            }
            pickup.stop();

            // Dropoff
            while (opModeIsActive() && !gamepad1.square);
            dropoff.start();
            while (opModeIsActive() && dropoff.update()) {
                robot.localization.update();
            }
            dropoff.stop();
        }
    }


    private void initSynchronizers() {

        // PICKUP

        // Elbow plan
        LinearElbow elbow1 = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow2 = new LinearElbow(elbow1.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan_pickup = new ElbowPlan(robot,
                elbow1,
                elbow2
        );

        // Claw plan
        LinearClaw claw1 = new LinearClaw(elbow1.getEndTime()-0.15,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ClawPlan clawPlan_pickup = new ClawPlan(robot,
                claw1
        );

        // Synchronizer
        this.pickup = new Synchronizer(
                clawPlan_pickup,
                elbowPlan_pickup
        );


        // DROPOFF

        // Elbow plan
        LinearElbow elbow3 = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow4 = new LinearElbow(elbow3.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan_dropoff = new ElbowPlan(robot,
                elbow3,
                elbow4
        );

        // Claw plan
        LinearClaw claw2 = new LinearClaw(elbow3.getEndTime()-0.15,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ClawPlan clawPlan_dropoff = new ClawPlan(robot,
                claw2
        );

        this.dropoff = new Synchronizer(
                clawPlan_dropoff,
                elbowPlan_dropoff
        );


    }

}
