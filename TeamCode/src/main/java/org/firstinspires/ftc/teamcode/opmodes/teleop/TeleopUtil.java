package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class TeleopUtil {
    private boolean isRedTeam;

    private final double CLAW_OPEN = ClawConstants.OPEN_POSITION;
    private final double CLAW_CLOSED = ClawConstants.CLOSED_POSITION;

    private final double ELBOW_UP = ElbowConstants.UP_POSITION;
    private final double ELBOW_DOWN = ElbowConstants.DOWN_POSITION;

    private final double SPEED_FACTOR = 1.0 / 6000;

    private double elbowPosition = ELBOW_UP;
    private boolean claw = false, toggleClaw = false; // false = claw open
    private boolean toggleMacro = false; //false = not in picking up position
    private boolean toggleMacroBasket = false;//false = not in reaching mode
    private double liftTargetPosition = 0;  //set lift pos to 0;
    private boolean liftMacroRunning = false; //while liftMacroRunning is true, other acts are not allowed during the movement

    private Gamepad gamepad1, gamepad2;
    private RobotSystem robot;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;


    public TeleopUtil(boolean isRedTeam, LinearOpMode opMode) {
        this.isRedTeam = isRedTeam;
        gamepad1 = opMode.gamepad1; gamepad2 = opMode.gamepad2;

        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new RobotSystem(opMode.hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, opMode);

        telemetry.setMsTransmissionInterval(11);

        robot.inDep.setClawPosition(CLAW_OPEN);
        robot.inDep.setElbowPosition(elbowPosition);
    }

    public void tick() throws InterruptedException {

        ////////////////////
        // DRIVE CONTROLS //
        ////////////////////

        // Turn speed down proportional to lift height
        double speed = 1 - Math.max(0, robot.inDep.getLeftLiftPosition() * SPEED_FACTOR);

        double forward = -speed * gamepad1.left_stick_y;
        double strafe = -speed * gamepad1.left_stick_x;
        double turn = speed * gamepad1.right_stick_x;

        double cvMacroMaxSpeed = 0.8;

        Set<CVSubsystem.Color> cvColors = new HashSet<>();
        boolean cvRunning = false;  // not defining cvRunning later for neatness and consistency in code
        cvColors.add(isRedTeam ? CVSubsystem.Color.RED : CVSubsystem.Color.BLUE);
        cvRunning |= robot.cv.tickTowardsSample(gamepad1.cross, cvMacroMaxSpeed, cvColors);
        cvColors.add(CVSubsystem.Color.YELLOW);
        cvRunning |= robot.cv.tickTowardsSample(gamepad1.cross && gamepad1.dpad_left, cvMacroMaxSpeed, cvColors);

        // if cv is running then drive will be determined the tickTowardsSample method
        if (!cvRunning) robot.drive.driveFieldCentricPowers(forward, strafe, turn, Math.toDegrees(robot.localization.getH()));

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);
        telemetry.addData("HEADING", robot.localization.getH());
        telemetry.update();


        ////////////////////
        // INDEP CONTROLS //
        ////////////////////

        // triangle = claw
        if (gamepad1.triangle && !toggleClaw) {
            toggleClaw = true;
            claw = !claw;
        }
        if (!gamepad1.triangle) toggleClaw = false;

        // Control claw
        if (claw)
            robot.inDep.setClawPosition(CLAW_CLOSED);
        else
            robot.inDep.setClawPosition(CLAW_OPEN);

        // RB: Raise elbow
        // LB: Lower elbow
        double friction = 0.03;
        if (gamepad1.right_bumper) elbowPosition += friction*(ELBOW_UP -elbowPosition);
        if (gamepad1.left_bumper) elbowPosition += friction*(ELBOW_DOWN -elbowPosition);

        // Control elbow
        robot.inDep.setElbowPosition(elbowPosition);


        ///////////////////
        // LIFT CONTROLS //
        ///////////////////

        // Left Lift
        //  - Square: negative power
        //  - Circle: positive power
        //
        // Right Lift
        //  - Cross: negative power
        //  - Trngl: positive power
        //
        // Both Motors
        //  - Left Trigger: negative power
        //  - Right Trigger: positive power

        double leftPower = 0;
        double rightPower = 0;
        double kP = 0.01;

        // Read gamepad
        double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        leftPower += triggerPower;
        rightPower += triggerPower;

        if (gamepad1.square && !toggleMacro) {
            claw = false;
            liftTargetPosition = 0;
            liftMacroRunning = true;
            toggleMacro = true;
        }
        if (!gamepad1.square)
            toggleMacro = false;

        if (!liftMacroRunning) {
            // Set powers
            robot.inDep.setLeftLiftPower(leftPower);
            robot.inDep.setRightLiftPower(rightPower);
        } else {
            //p controller
            double liftPosition = robot.inDep.getLeftLiftPosition(); //get current position
            double error = liftTargetPosition - liftPosition;

            //control law
            double u_t = kP * error;
            robot.inDep.setLeftLiftPower(u_t);
            robot.inDep.setRightLiftPower(u_t);

            if (Math.abs(error) < 50) {
                liftMacroRunning = false;
            }
        }

        if (gamepad1.right_stick_button && !toggleMacroBasket) {
            triggerPower = 20; //lift goes up
            sleep(5000);
            claw = true; //claw open
            toggleMacroBasket = !toggleMacroBasket; //set toggle to true
        }
        if (!gamepad1.right_stick_button) {
            toggleMacroBasket = false;
        }
    }
}
