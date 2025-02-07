package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;

import java.util.ArrayDeque;

@Autonomous(name="Dynamic Extendo Spam Test", group = "Calibration")
public class DynamicExtendoSpamTest extends LinearOpMode {

    private Synchronizer synchronizer;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private LinearSlidesSubsystem linearSlides;

    private boolean extending = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        ExtendoState position = new ExtendoState(linearSlides.getExtendoPosition());
        ExtendoState velocity = new ExtendoState(0);
        initSynchronizer(position, velocity);

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        boolean toggleSquare = false;

        while (opModeIsActive() && gamepad1.square) updateTPS();
        while (opModeIsActive() && !gamepad1.square) updateTPS();
        synchronizer.start();
        while (opModeIsActive()) {
            synchronizer.update();
            updateTPS();

            // square is spammable hopefully
            if (gamepad1.square && !toggleSquare) {
                extending = !extending;
                position = (ExtendoState) synchronizer.getState(MovementType.EXTENDO);
                velocity = (ExtendoState) synchronizer.getVelocity(MovementType.EXTENDO);
                initSynchronizer(position, velocity);
                synchronizer.start();
                toggleSquare = true;
            }
            if (!gamepad1.square) toggleSquare = false;
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init linear slides
        Motor extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620);
        Motor leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        Motor rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(true);

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

        this.linearSlides = new LinearSlidesSubsystem(extendo, leftLift, rightLift, telemetry);
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.update();
    }

    private void initSynchronizer(ExtendoState position, ExtendoState velocity) {
        double extendoTarget; // inches
        if (extending) extendoTarget = 24;
        else extendoTarget = 0;

        // Extendo
        DynamicLinearExtendo extendo = new DynamicLinearExtendo(0,
                position,
                new ExtendoState(extendoTarget),
                velocity
        );

        // Create Plans
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendo
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                extendo_plan
        );
    }
}
