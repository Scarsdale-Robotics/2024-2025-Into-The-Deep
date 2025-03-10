package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.GrabHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;

import java.util.ArrayDeque;

@Autonomous(name="Horizontal Intake Pick Up Sample Motion", group = "Calibration")
public class HIntakePickUpSampleMotion extends LinearOpMode {

    private Synchronizer synchronizer1;
    private Synchronizer synchronizer2;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        initSynchronizer1();
        initSynchronizer2();


        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !gamepad1.square) updateTPS();
        while (opModeIsActive()) {
            // pick up
            synchronizer1.start();
            while (opModeIsActive() && synchronizer1.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.square) {
                updateTPS();
                synchronizer1.update();
            }
            synchronizer1.stop();
            updateTPS();
            // drop off
            synchronizer2.start();
            while (opModeIsActive() && synchronizer2.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.square) {
                updateTPS();
                synchronizer2.update();
            }
            synchronizer2.stop();
            updateTPS();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo leftHorizontalArm = hardwareMap.get(ServoImplEx.class, "leftHorizontalArm");
        Servo rightHorizontalArm = hardwareMap.get(ServoImplEx.class, "rightHorizontalArm");
        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");
        Servo horizontalClaw = hardwareMap.get(ServoImplEx.class, "horizontalClaw");
        this.horizontalIntake = new HorizontalIntakeSubsystem(
                leftHorizontalArm,
                rightHorizontalArm,
                horizontalWrist,
                horizontalClaw
        );
        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.5);
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.update();
    }

    private void initSynchronizer1() {
        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(0,
                new HArmState(0.5),
                new HArmState(1.05)
        );
        MoveHWrist h_wrist_align = new MoveHWrist(0, Math.toRadians(0));
        ReleaseHClaw h_claw_release = new ReleaseHClaw(0);

        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime());
        MoveHWrist h_wrist_reset = new MoveHWrist(h_claw_grab.getEndTime(), 0);
        LinearHArm h_arm_up = new LinearHArm(h_wrist_reset.getEndTime(),
                new HArmState(1.05),
                new HArmState(0.5)
        );

        // Create Plans
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_release,
                h_claw_grab
        );

        // Synchronizer
        this.synchronizer1 = new Synchronizer(
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }

    private void initSynchronizer2() {
        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(0,
                new HArmState(0.5),
                new HArmState(1.05)
        );
        MoveHWrist h_wrist_align = new MoveHWrist(0, Math.toRadians(0));
        GrabHClaw h_claw_grab = new GrabHClaw(0);

        // Pick up and move arm up
        ReleaseHClaw h_claw_release = new ReleaseHClaw(h_arm_down.getEndTime());
        MoveHWrist h_wrist_reset = new MoveHWrist(h_claw_release.getEndTime(), 0);
        LinearHArm h_arm_up = new LinearHArm(h_wrist_reset.getEndTime(),
                new HArmState(1.05),
                new HArmState(0.5)
        );

        // Create Plans
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_release,
                h_claw_grab
        );

        // Synchronizer
        this.synchronizer2 = new Synchronizer(
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }
}
