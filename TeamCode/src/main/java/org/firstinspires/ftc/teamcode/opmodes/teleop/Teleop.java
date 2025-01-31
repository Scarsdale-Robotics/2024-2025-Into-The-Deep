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
            InDepSubsystem.InDepControlData inDepControls = new InDepSubsystem.InDepControlData();

            inDepControls.depositButton = !gamepad2.dpad_left && gamepad2.triangle;
            inDepControls.intakeButton = !gamepad1.dpad_left && gamepad1.triangle;
            inDepControls.magButton = !gamepad2.dpad_left && gamepad2.square;
            inDepControls.makerButton = !gamepad2.dpad_left && gamepad2.circle;
            inDepControls.clipButton = !gamepad1.dpad_left && gamepad1.square;

            inDepControls.intakeUndo = gamepad2.dpad_left && inDepControls.depositButton;
            inDepControls.depositUndo = gamepad1.dpad_left && inDepControls.intakeButton;
            inDepControls.magUndo = gamepad2.dpad_left && inDepControls.magButton;
            inDepControls.makerUndo = gamepad2.dpad_left && inDepControls.makerButton;
            inDepControls.clipUndo = gamepad1.dpad_left && inDepControls.clipButton;

            double g1power = gamepad1.right_trigger - gamepad1.left_trigger;
            double g2power = gamepad2.right_trigger - gamepad2.left_trigger;

            inDepControls.intakePower = g1power;
            inDepControls.depositPower = g2power;

            inDepControls.intakeSemidirectCD.intakeLiftPower = gamepad1.dpad_up ? g1power : 0;
                inDepControls.intakeDirectCD.intakeLeftLiftPower = (gamepad1.dpad_up && gamepad1.left_bumper) ? g1power : 0;
                inDepControls.intakeDirectCD.intakeRightLiftPower = (gamepad1.dpad_up && gamepad1.right_bumper) ? g1power : 0;
            inDepControls.intakeSemidirectCD.intakeWristPower = gamepad1.dpad_left ? g1power : 0;
                inDepControls.intakeDirectCD.intakeWristPower = inDepControls.intakeSemidirectCD.intakeWristPower;
            inDepControls.intakeSemidirectCD.intakePivotPower = gamepad1.dpad_down ? g1power : 0;
                inDepControls.intakeDirectCD.intakePivotPower = inDepControls.intakeSemidirectCD.intakePivotPower;
            inDepControls.intakeSemidirectCD.intakeClawPower = gamepad1.dpad_right ? g1power : 0;
                inDepControls.intakeDirectCD.intakeClawPower = inDepControls.intakeSemidirectCD.intakeClawPower;

            inDepControls.depositSemidirectCD.depositSlidePower = gamepad2.dpad_up ? g2power : 0;
                inDepControls.depositDirectCD.depositLeftSlidePower = (gamepad2.dpad_up && gamepad2.left_bumper) ? g2power : 0;
                inDepControls.depositDirectCD.depositRightSlidePower = (gamepad2.dpad_up && gamepad2.right_bumper) ? g2power : 0;
            inDepControls.depositSemidirectCD.depositWristPower = gamepad2.dpad_left ? g2power : 0;
                inDepControls.depositDirectCD.depositWristPower = inDepControls.depositSemidirectCD.depositWristPower;
            inDepControls.depositSemidirectCD.depositClawPower = gamepad2.dpad_right ? g2power : 0;
                inDepControls.depositDirectCD.depositClawPower = inDepControls.depositSemidirectCD.depositClawPower;

            inDepControls.magazineSemidirectCD.magServoPower = gamepad2.square ? g2power : 0;
                inDepControls.magazineDirectCD.magServoPower = inDepControls.magazineSemidirectCD.magServoPower;

            inDepControls.makerSemidirectCD.makerServoPower = gamepad2.circle ? g2power : 0;
                inDepControls.makerDirectCD.makerServoPower = inDepControls.makerSemidirectCD.makerServoPower;

            inDepControls.clipSemidirectCD.clipIntakeServoPower = gamepad1.square ? g1power : 0;
                inDepControls.clipDirectCD.clipIntakeServoPower = inDepControls.clipSemidirectCD.clipIntakeServoPower;
        }
    }

}
