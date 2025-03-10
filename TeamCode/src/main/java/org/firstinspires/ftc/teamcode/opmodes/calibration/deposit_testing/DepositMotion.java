package org.firstinspires.ftc.teamcode.opmodes.calibration.deposit_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

import java.util.ArrayDeque;

@Config
@TeleOp(name = "Deposit Motion", group = "Calibration")
public class DepositMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private AutonomousRobot robot;

    public static double liftDownDelay = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        initSynchronizer();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) updateTPS();
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(),
                // back against wall, facing towards sub, centered on first seam from middle
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );

        robot.verticalDeposit.setArmPosition(VArmConstants.armLeftPreDepositPosition);
        robot.verticalDeposit.setClawPosition(VClawConstants.GRAB_POSITION);
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        robot.update();
        telemetry.update();
    }

    private void initSynchronizer() {
        LinearLift liftToPreDeposit = new LinearLift(0,
                new LiftState(0),
                new LiftState(LiftConstants.preDepositPosition)
        );

        LinearVArm armToDeposit = new LinearVArm(liftToPreDeposit.getEndTime(),
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift liftToDeposit = new LinearLift(armToDeposit.getEndTime(),
                new LiftState(LiftConstants.preDepositPosition),
                new LiftState(LiftConstants.depositPosition)
        );

        ReleaseVClaw releaseVClaw = new ReleaseVClaw(liftToDeposit.getEndTime());

        LinearLift liftDown = new LinearLift(releaseVClaw.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );


        // Plans
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftToPreDeposit,
                liftToDeposit,
                liftDown
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                armToDeposit
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                releaseVClaw
        );


        // Synchronizer
        synchronizer = new Synchronizer(
                liftPlan,
                vArmPlan,
                vClawPlan
        );
    }
}
