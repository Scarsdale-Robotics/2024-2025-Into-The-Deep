package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;

@Config
@TeleOp(name="Blue TeleOp")
public class BlueTeleop extends LinearOpMode {

    private RobotSystem robot;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;
    private double elbowPosition = elbowUp;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new RobotSystem(hardwareMap, new Pose2d(48, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);

        robot.inDep.setClawPosition(clawClosed);
        robot.inDep.setElbowPosition(elbowPosition);

        waitForStart();

        double speed = 1;
        boolean claw = true, toggleClaw = false; // false = claw open
        double liftTargetPosition = 0;  //set lift pos to 0;

        boolean toggleLiftUpMacro = false, // Lifting to submersible high chamber
                liftUpMacroRunning = false;

        boolean toggleLiftDownMacro = false, // Go down to specimen
                liftDownMacroRunning = false;

        boolean autoClawReleaseAvailable = false;

        while (opModeIsActive()) {
            robot.localization.update();
            robot.logOdometry();

            // options+circle (gamepad2) = Reset IMU
            if (gamepad2.square && gamepad2.circle) {
                robot.localization.resetH(Math.toRadians(-90));
                gamepad1.rumble(1500);
                gamepad2.rumble(1500);
            }

            ////////////////////
            // DRIVE CONTROLS //
            ////////////////////

            // Turn speed down proportional to lift height
            speed = 1 - Math.max(0,robot.inDep.getLeftLiftPosition()/6000d);

            double speedArmMultiplier;
            double elbowProportionalPosition = (elbowPosition - elbowDown) / (elbowUp - elbowDown);
            if (elbowProportionalPosition < 0.2) speedArmMultiplier = 0.3;
            else speedArmMultiplier = 1;

            double forward = speedArmMultiplier * -speed * gamepad1.left_stick_y;
            double strafe = speedArmMultiplier * -speed * gamepad1.left_stick_x;
            double turn = speedArmMultiplier * speed * gamepad1.right_stick_x;

            robot.drive.driveFieldCentricPowers(forward, strafe, turn, Math.toDegrees(robot.localization.getH()));
            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
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
                robot.inDep.setClawPosition(clawClosed);
            else
                robot.inDep.setClawPosition(clawOpen);


            // RB: Raise elbow
            // LB: Lower elbow
            double slowElbowSpeed = 0.0025;
            double fastElbowSpeed = 0.01;
            double speedThreshold = 0.5; // proportion between up and down
            double speedGradient = slowElbowSpeed+(elbowProportionalPosition-speedThreshold)*(fastElbowSpeed-slowElbowSpeed); // linearly connect speeds when elbow is higher than threshold

            if (gamepad1.right_bumper) {
                if (elbowProportionalPosition < speedThreshold)
                    elbowPosition = Math.max(elbowUp, elbowPosition - slowElbowSpeed);
                else
                    elbowPosition = Math.max(elbowUp, elbowPosition - speedGradient);
            }

            if (gamepad1.left_bumper) {
                if (elbowProportionalPosition < speedThreshold)
                    elbowPosition = Math.min(elbowDown, elbowPosition + slowElbowSpeed);
                else
                    elbowPosition = Math.min(elbowDown, elbowPosition + speedGradient);
            }

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



            // LIFT UP MACRO
            if (gamepad1.square && !toggleLiftUpMacro) {
                liftTargetPosition = 1350;
                liftUpMacroRunning = true; // enable lift up macro
                liftDownMacroRunning = false; // disable lift down macro
                toggleLiftUpMacro = true;
                elbowPosition = elbowUp;
            }
            if (!gamepad1.square)
                toggleLiftUpMacro = false;


            // LIFT DOWN MACRO
            if (gamepad1.circle && !toggleLiftDownMacro) {
                liftTargetPosition = -10;
                liftUpMacroRunning = false; // disable lift up macro
                liftDownMacroRunning = true; // enable lift down macro
                toggleLiftDownMacro = true;
                elbowPosition = elbowDown - 0.04;
            }
            if (!gamepad1.circle)
                toggleLiftDownMacro = false;


            // CONTROL LIFT
            boolean triggerPressed = gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0;
            if (triggerPressed) {
                liftUpMacroRunning = false;
                liftDownMacroRunning = false;
            }
            if (liftUpMacroRunning || liftDownMacroRunning) {
                // A MACRO IS RUNNING
                //p controller
                double liftPosition = robot.inDep.getLeftLiftPosition(); //get current position
                double error = liftTargetPosition - liftPosition;

                //control law
                double u_t = kP * error;
                robot.inDep.setLeftLiftPower(u_t);
                robot.inDep.setRightLiftPower(u_t);

                if (Math.abs(error) < 50) {
                    if (liftUpMacroRunning) autoClawReleaseAvailable = true;
                    liftUpMacroRunning = false;
                    liftDownMacroRunning = false;
                }
            }
            else {
                // TRIGGER CONTROLS
                // Set powers
                robot.inDep.setLeftLiftPower(leftPower);
                robot.inDep.setRightLiftPower(rightPower);
                if (autoClawReleaseAvailable && robot.inDep.getLeftLiftPosition() < 650) {
                    claw = false;
                    robot.inDep.setClawPosition(clawOpen);
                    autoClawReleaseAvailable = false;
                }
            }




        }

    }

}
/**
 * Clamp the input power between [-1,1]
 * @return Math.max(-1, Math.min(1, x));
 */
       	/* private double clamp (double power){
            	return Math.max(-1, Math.min(1, power));
        	}
    	}
*/



