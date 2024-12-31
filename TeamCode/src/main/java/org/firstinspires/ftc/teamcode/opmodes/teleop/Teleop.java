package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.indep.InDepSubsystem;

public class Teleop extends LinearOpMode {
    private RobotSystem robot;
    private InDepSubsystem indep;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);

        this.indep = robot.inDep;

        waitForStart();

        while (opModeIsActive()) {
            // TODO: INDEP UNDOs
            InDepSubsystem.InDepControlData inDepControls = new InDepSubsystem.InDepControlData();
            inDepControls.set(
                    gamepad1.triangle,
                    gamepad2.triangle,
                    gamepad2.square,
                    gamepad1.circle,
                    gamepad1.square,
                    gamepad1.right_trigger - gamepad1.left_trigger,
                    gamepad2.right_trigger - gamepad2.left_trigger
            );
        }
    }

}
