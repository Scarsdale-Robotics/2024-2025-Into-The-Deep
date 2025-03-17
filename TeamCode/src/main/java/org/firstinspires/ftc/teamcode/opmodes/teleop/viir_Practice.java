package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;

@TeleOp(name = "ViirTeleop")
//hello just random stuff i do here to practice, i know a lot more than i can write trust.
//upate 3/9 on way back from bus: we clutched up. i learned odometry, localization, and cv in one bus reide les goooo
//TODO: Viir's code: add lift stuff.
//TODO: Viir's code: implement control theory in some code (error and stuff) for running more complicated macros
//IMPORTANT: don't merge changes to sync macro or main, private branch.
public class viir_Practice extends LinearOpMode {
    public static double clawClosed = ClawConstants.CLOSED_POSITION;
    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;
    public static double elbowPos = elbowUp;
    public static double clawPos = clawClosed;
    private RobotSystem robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);

        //telemetry stuff (find out meaning of multiple telemetry)
        robot.inDep.setClawPosition(clawPos);
        robot.inDep.setElbowPosition(elbowPos);
        waitForStart();
        while (opModeIsActive()) {
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setElbowPosition(elbowPos);
            String viir = "Sigma Sigma boy";
            robot.localization.update();
            robot.logOdometry();
            double speed = 0.9;
            //toggle boolean for displaying telemetry, drive powers (idk what scale the gamepad stuff is on), buffer for reducing sense
            boolean displayTelemetry = false;
            boolean isClawOpen = false;
            boolean toggleLiftUpMacro = false;
            boolean liftDownMacroRunning = false;
            boolean liftUpMacroRunning = false;
            boolean toggleLiftDownMacro = false;
            boolean isElbowDown = false;
            boolean senseChangeActivated = false;
            double strafe = gamepad1.left_stick_x * speed;
            double forward = -gamepad1.left_stick_y * speed;
            double turn = gamepad1.right_stick_x * speed;
            double buffer = 0.2;
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Buffer for sense change: ", buffer);
            telemetry.addData("Viir is a ", viir);
            telemetry.update();
            if (gamepad1.x) {
                isElbowDown = true;
            }
            if (isElbowDown) {
                elbowPos = elbowDown;
            }
            if (gamepad1.x && isElbowDown) {
                isElbowDown = false;
                elbowPos = elbowUp;
            }
            if (gamepad1.circle) {
                isClawOpen = true;
            }
            if (isClawOpen) {
                clawPos = clawOpen;
            }
            if (gamepad1.circle && isClawOpen) {
                isClawOpen = false;
                clawPos = clawClosed;
            }
            //field centric drive: includes angles for more accuracy with regards to human perspective
            //get H - get heading
            robot.drive.driveFieldCentric(strafe , forward, turn, Math.toDegrees(robot.localization.getH()));
            if (gamepad1.circle) {
                senseChangeActivated = true;
            }
            if (senseChangeActivated) {
                strafe = -gamepad1.left_stick_x * speed + buffer;
                turn = gamepad1.right_stick_x * speed - buffer;
                forward = gamepad1.left_stick_y * speed - buffer;
            }
            if (senseChangeActivated && gamepad1.circle) {
                senseChangeActivated = false;
                strafe = -gamepad1.left_stick_x * speed;
                turn = gamepad1.right_stick_x * speed;
                forward = gamepad1.left_stick_y * speed;
            }
            boolean speedUpToggled = false;
            if (gamepad1.a) {
                speedUpToggled = true;
            }
            if (speedUpToggled) {
                speed = 1;
            }
            if (gamepad1.a & speedUpToggled) {
                speedUpToggled = false;
                speed = 0.9;
            }
            if (gamepad1.dpad_down) {
                realignMacro();
            }
            if (gamepad1.dpad_up) {
                resetServos();
                telemetry.addLine("Reset of Claw and Elbow Successful.");
                telemetry.update();
            }
            while (!gamepad1.circle || !gamepad1.x) {
                clawPos = clawClosed;
                elbowPos = elbowUp;
            }
            //define lift powers
            double rightPowerLift = 0; //individual powers
            double leftPowerLift = 0;
            double kP = 0.01; //control theory/law or whatever its called
            double liftTargetPosition = 0; //macro target pos to get lift to
            //combine both triggers
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            //set both to difference
            rightPowerLift += triggerPower;
            leftPowerLift += triggerPower;
            if (gamepad1.dpad_left && !toggleLiftUpMacro) {
                toggleLiftUpMacro = true;
                liftTargetPosition = 1350;
                liftUpMacroRunning = true;
                liftDownMacroRunning = false;
                elbowPos = elbowUp;
            }
            if (!gamepad1.dpad_left) {
                toggleLiftUpMacro = false;
            }
            if (gamepad1.dpad_right && !toggleLiftDownMacro) {
                liftTargetPosition = -10; //target pos aka up lift position
                liftUpMacroRunning = false;
                toggleLiftDownMacro = true;
                liftDownMacroRunning = true;
                elbowPos = elbowDown - 0.04;
            }
            if (!gamepad1.dpad_right) {
                toggleLiftDownMacro = false;
            }
            boolean triggerPressed = gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0;
            if (triggerPressed) {
                liftUpMacroRunning = false;
                liftDownMacroRunning = false;
            }
            if (liftUpMacroRunning || liftDownMacroRunning) {
                double liftPosition = robot.inDep.getLeftLiftPosition();
                double error = liftTargetPosition - liftPosition;
                double u_t = kP * error; //random calc stuff idk
                robot.inDep.setLeftLiftPower(u_t);
                robot.inDep.setRightLiftPower(u_t);
                if (Math.abs(error) < 50) {
                    liftUpMacroRunning = false;
                    liftDownMacroRunning = false;
                }
                //above stops the macro if error gets too small (lift is close to target pos)
            }
            //otherwise, set the lift positions to the active inputs.
            //the lift up and lift down macros are for hanging specimens.
            // to hang spec, lift up macro and then lift down macro.
            else {
                robot.inDep.setLeftLiftPower(leftPowerLift);
                robot.inDep.setRightLiftPower(rightPowerLift);
            }
        }
    }
    public void realignMacro() {
        robot.localization.resetH(Math.toRadians(-180));
        double currentHeading = robot.localization.getH();
        robot.localization.update();
        robot.logOdometry();
        telemetry.addData("Reset Sucessful. Heading: ", currentHeading);
    }
    public void resetServos() {
        clawPos = clawClosed;
        elbowPos = elbowUp;
    }
}