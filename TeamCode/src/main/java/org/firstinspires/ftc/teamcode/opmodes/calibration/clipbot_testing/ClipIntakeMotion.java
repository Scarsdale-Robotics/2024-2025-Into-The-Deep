package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakePlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.MoveMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;

import java.util.ArrayDeque;

@Autonomous(name="Clip Intake Motion", group = "Calibration")
public class ClipIntakeMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private Servo magazineIntake;
    private Servo magazineLoader;
    private Motor magazineFeeder;

    private ClipbotSubsystem clipbot;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        initSynchronizer();

        clipbot.setMagazineIntakePosition(MIntakeConstants.outPosition);
        clipbot.setMagazineLoaderPosition(MLoaderConstants.upPosition);

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !gamepad1.square) updateTPS();
        while (opModeIsActive()) {
            // pick up
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.square) {
                updateTPS();
                synchronizer.update();
            }
            synchronizer.stop();
            updateTPS();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        magazineIntake = hardwareMap.get(ServoImplEx.class, "magazineIntake");
        magazineLoader = hardwareMap.get(ServoImplEx.class, "magazineLoader");

        magazineFeeder = new MotorEx(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620);
        magazineFeeder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineFeeder.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazineFeeder.setRunMode(Motor.RunMode.RawPower);
        magazineFeeder.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setInverted(false);

        this.clipbot = new ClipbotSubsystem(
                magazineIntake,
                magazineLoader,
                magazineFeeder,
                telemetry
        );
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        clipbot.update();
        telemetry.update();
    }

    private void initSynchronizer() {

        // servo starting positions
        double startTime = 0;
        MoveMIntake intakeStart = new MoveMIntake(new TimeSpan(startTime,startTime),
                MIntakeConstants.outPosition
        );
        MoveMLoader loaderStart = new MoveMLoader(new TimeSpan(startTime,startTime),
                MLoaderConstants.upPosition
        );

        // intake and load clips
        MoveMIntake intakeIn = new MoveMIntake(startTime,
                MIntakeConstants.inPosition
        );
        MoveMLoader loaderDown = new MoveMLoader(intakeIn.getEndTime(),
                MLoaderConstants.downPosition
        );

        // reset loader then intake
        MoveMLoader loaderUp = new MoveMLoader(loaderDown.getEndTime(),
                MLoaderConstants.upPosition
        );
        MoveMIntake intakeOut = new MoveMIntake(loaderUp.getEndTime(),
                MIntakeConstants.outPosition
        );


        // Plans
        MIntakePlan mIntakePlan = new MIntakePlan(clipbot,
                intakeStart,
                intakeIn,
                intakeOut
        );

        MLoaderPlan mLoaderPlan = new MLoaderPlan(clipbot,
                loaderStart,
                loaderDown,
                loaderUp
        );


        // Synchronizer
        this.synchronizer = new Synchronizer(
                mIntakePlan,
                mLoaderPlan
        );
    }

}
