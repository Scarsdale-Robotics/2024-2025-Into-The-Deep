package org.firstinspires.ftc.teamcode.opmodes.calibration.outtake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;

import java.util.ArrayDeque;

@Autonomous(name="Debugging SynchroPather Lift OpMode", group = "Calibration")
public class DebuggingSynchroPatherLift extends LinearOpMode {

    Synchronizer synchronizer;

    volatile ArrayDeque<Double> loopTicks;
    volatile ElapsedTime runtime;

    private LinearSlidesSubsystem linearSlides;
    private MotorEx leftLift;
    private MotorEx rightLift;

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

        while (opModeIsActive()) {
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                updateTPS();
                telemetry.addData("status", "running");
                telemetry.update();
            }
            while (opModeIsActive() && !gamepad1.square) {
                updateTPS();
                synchronizer.update();
                telemetry.addData("status", "stopped");
                telemetry.update();
            }
            synchronizer.stop();
            updateTPS();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(false);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);

        this.linearSlides = new LinearSlidesSubsystem(
                null,
                leftLift,
                rightLift,
                telemetry
        );
    }

    private void updateTPS() {
        /////////////////
        // TPS COUNTER //
        /////////////////

        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.update();
        linearSlides.update();
    }


    private void initSynchronizer() {
        double targetHeight = 0.65*LiftConstants.MAX_HEIGHT;

        // Translation plan
        LinearLift lift1 = new LinearLift(0,
                new LiftState(0),
                new LiftState(targetHeight)
        );
        telemetry.addData("lift1.getDuration()", lift1.getDuration());
        telemetry.addData("lift1.getTimeSpan()", lift1.getTimeSpan().toString());
        LinearLift lift2 = new LinearLift(lift1.getEndTime(),
                new LiftState(targetHeight),
                new LiftState(0),
                true
        );
        telemetry.addData("lift2.getDuration()", lift2.getDuration());
        LiftPlan liftPlan = new LiftPlan(linearSlides,
                lift1,
                lift2
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                liftPlan
        );
        telemetry.addData("synchronizer.getDuration()", synchronizer.getDuration());
        telemetry.update();
    }

}
