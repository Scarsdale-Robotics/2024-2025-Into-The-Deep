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
@TeleOp(name="Red TeleOp")
public class RedTeleop extends LinearOpMode {

    private RobotSystem robot;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;
    private double elbowPosition = elbowUp;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), true, this);

        robot.inDep.setClawPosition(clawOpen);
        robot.inDep.setElbowPosition(elbowPosition);

        waitForStart();

        double speed = 1;
        boolean claw = false, toggleClaw = false; // false = claw open
        boolean toggleMacro = false; //false = not in picking up position
        boolean toggleMacroBasket = false;//false = not in reaching mode
        double liftTargetPosition = 0;  //set lift pos to 0;
        boolean liftMacroRunning = false; //while liftMacroRunning is true, other acts are not allowed during the movement

        while (opModeIsActive()) {
            robot.localization.update();

            ////////////////////
            // DRIVE CONTROLS //
            ////////////////////

            // Turn speed down proportional to lift height
            speed = 1 - Math.max(0,robot.inDep.getLeftLiftPosition()/6000d);

            double forward = -speed * gamepad1.left_stick_y;
            double strafe = -speed * gamepad1.left_stick_x;
            double turn = speed * gamepad1.right_stick_x;

            robot.drive.driveFieldCentricPowers(forward, strafe, turn, Math.toDegrees(robot.localization.getH()));
            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.addData("HEADING", robot.localization.getH());
            telemetry.addData("speed", speed);
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
            double friction = 0.04;
            if (gamepad1.right_bumper) elbowPosition += friction*(elbowUp-elbowPosition);
            if (gamepad1.left_bumper) elbowPosition += friction*(elbowDown-elbowPosition);

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

            // Clamp powers
        	  /*
        	  leftPower = clamp(leftPower);
        	  rightPower = clamp(rightPower);
        	  */


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
            /*else if (gamepad1.right_stick_button && toggleMacroBasket){
                triggerPower = -20;
                sleep(5000);
                claw = true; //claw open
                toggleMacroBasket = !toggleMacroBasket;
            } */
            if(!gamepad1.right_stick_button) {
                toggleMacroBasket = false;
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



