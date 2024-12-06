package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;

@TeleOp(name="Touchpad Test", group="Calibration")
public class TouchpadTest extends LinearOpMode {

    RobotSystem robot;
    Synchronizer pickup, dropoff;

    double globalX = 0, globalY = 0, globalH = Math.toRadians(90);
    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION-0.04;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotSystem(hardwareMap, new Pose2d(globalX,globalY,new Rotation2d(globalH)), false, this);
        robot.inDep.setClawPosition(clawOpen);
        robot.inDep.setElbowPosition(elbowUp-0.04);
        initSynchronizers();

        waitForStart();

        boolean dragging1 = false, dragging2 = false;
        double touchpadMultiplier = 5;
        double clickX = 0, clickY = 0, clickH = 0;
        double dragDeltaX = 0, dragDeltaY = 0, dragDeltaH = 0;
        double clickGlobalX = 0, clickGlobalY = 0, clickGlobalH = 0;

        boolean indepActive = false;
        boolean pickupOrDropoff = false; // false = dropoff, true = pickup

        while (opModeIsActive()) {
            robot.localization.update();

            // Sense touchpad
            boolean touchpadF1 = gamepad1.touchpad_finger_1;
            double touchpadF1X = touchpadMultiplier * gamepad1.touchpad_finger_1_x;
            double touchpadF1Y = touchpadMultiplier * gamepad1.touchpad_finger_1_y;
            boolean touchpadF2 = gamepad1.touchpad_finger_2;
            double touchpadF2X = touchpadMultiplier * gamepad1.touchpad_finger_2_x;
            double touchpadF2Y = touchpadMultiplier * gamepad1.touchpad_finger_2_y;
            boolean touchpad  = gamepad1.touchpad;

            telemetry.addData("touchpadF1", touchpadF1);
            telemetry.addData("touchpadF1X", touchpadF1X);
            telemetry.addData("touchpadF1Y", touchpadF1Y);
            telemetry.addData("touchpadF2", touchpadF2);
            telemetry.addData("touchpadF2X", touchpadF2X);
            telemetry.addData("touchpadF2Y", touchpadF2Y);

            // Sense dragging
            // One finger = translation
            if (touchpadF1 && !touchpadF2) {
                if (!dragging1) {
                    dragging1 = true;
                    clickX = touchpadF1X;
                    clickY = touchpadF1Y;
                    clickGlobalX = globalX;
                    clickGlobalY = globalY;
                }
                dragDeltaX = touchpadF1X - clickX;
                dragDeltaY = touchpadF1Y - clickY;
                globalX = clickGlobalX + dragDeltaX;
                globalY = clickGlobalY + dragDeltaY;
            } else {
                dragging1 = false;
            }

            // Two fingers = rotation
            if (touchpadF1 && touchpadF2) {
                double theta = Math.atan2(touchpadF2Y - touchpadF1Y, touchpadF2X - touchpadF1X);
                if (!dragging2) {
                    dragging2 = true;
                    clickH = theta;
                    clickGlobalH = globalH;
                }
                dragDeltaH = normalizeAngle(theta - clickH);
                globalH = clickGlobalH + dragDeltaH;
            } else {
                dragging2 = false;
            }

            telemetry.addData("dragging1", dragging1);
            telemetry.addData("dragging2", dragging2);
            telemetry.addData("clickX", clickX);
            telemetry.addData("clickY", clickY);
            telemetry.addData("clickH", clickH);
            telemetry.addData("dragDeltaX", dragDeltaX);
            telemetry.addData("dragDeltaY", dragDeltaY);
            telemetry.addData("dragDeltaH", dragDeltaH);
            telemetry.addData("globalX", globalX);
            telemetry.addData("globalY", globalY);
            telemetry.addData("globalH", globalH);


            // Click for picking up
            if (touchpad && !indepActive) {
                indepActive = true;
                pickupOrDropoff = !pickupOrDropoff;
                if (pickupOrDropoff)
                    pickup.start();
                else
                    dropoff.start();
            }

            telemetry.addData("indepActive", indepActive);
            telemetry.addData("pickupOrDropoff", pickupOrDropoff);


            if (indepActive) {
                if (pickupOrDropoff) {
                    if (!pickup.update()) {
                        pickup.stop();
                        indepActive = false;
                    }
                }
                else {
                    if (!dropoff.update()) {
                        dropoff.stop();
                        indepActive = false;
                    }
                }
            }




            // Correct to pose

            double kP = 0.25;
            double[] error = {
                    globalX - robot.localization.getX(),
                    globalY - robot.localization.getY(),
                    normalizeAngle(globalH - robot.localization.getH())
            };

            telemetry.addData("error[0]", error[0]);
            telemetry.addData("error[1]", error[1]);
            telemetry.addData("error[2]", error[2]);

            double x = 0, y = 0, turn = 0;
            x += kP * error[0];
            y += kP * error[1];
            turn += 16 * kP * error[2];

            telemetry.addData("output x", x);
            telemetry.addData("output y", y);
            telemetry.addData("output turn", turn);

            robot.drive.driveFieldCentricPowers(x, y, -turn, robot.localization.getH());

            telemetry.update();
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

    /**
     * Normalizes a given angle to [-pi,pi) degrees.
     * @param degrees the given angle in degrees.
     * @return the normalized angle in degrees.
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (opModeIsActive() && angle <= -Math.PI)
            angle += 2*Math.PI;
        while (opModeIsActive() && angle > Math.PI)
            angle -= 2*Math.PI;
        return angle;
    }

}
