package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements.MoveKlipper;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.MoveVClaw;

import java.util.ArrayDeque;

@Config
@Autonomous(name="Klipper motion", group = "Calibration")
public class KlipperMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private Servo magazineIntake;
    private Servo magazineLoader1;
    private Servo magazineLoader2;
    private Motor magazineFeeder;

    private ClipbotSubsystem clipbot;
    public VerticalDepositSubsystem verticalDeposit;
    private LinearSlidesSubsystem linearSlides;

    private int clipInventory = MFeederConstants.RELOAD_CAPACITY;
    private boolean inventoryStocked = true;

    public static double clawGrabWaitTime = 1;
    public static double klipperWaitTime = 0.5;
    public static double feederDelayTime = 0.5;
    public static double holdLiftDownPosition = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Wait for button press
            while (opModeIsActive() && (gamepad1.triangle || !inventoryStocked)) {
                updateTPS();
                if (gamepad1.square) {
                    inventoryStocked = true;
                    clipInventory = MFeederConstants.RELOAD_CAPACITY;
                }
            }
            while (opModeIsActive() && !gamepad1.triangle) updateTPS();
            if (clipInventory <= 0) continue;
            initAdvanceMagazineMotion();

            // pick up
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.triangle) {
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
        magazineLoader1 = hardwareMap.get(ServoImplEx.class, "magazineLoaderClose");
        magazineLoader2 = hardwareMap.get(ServoImplEx.class, "magazineLoaderFar");

        magazineFeeder = new MotorEx(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620);
        magazineFeeder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineFeeder.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazineFeeder.setRunMode(Motor.RunMode.RawPower);
        magazineFeeder.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setInverted(false);

        this.clipbot = new ClipbotSubsystem(
                magazineIntake,
                magazineLoader1,
                magazineLoader2,
                initServo(hardwareMap, "klipper"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620),
                telemetry,
                this
        );




        // init vertical deposit
        Servo leftDepositArm = hardwareMap.get(ServoImplEx.class, "leftDepositArm");
        Servo rightDepositArm = hardwareMap.get(ServoImplEx.class, "rightDepositArm");
        Servo depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");
        this.verticalDeposit = new VerticalDepositSubsystem(
                leftDepositArm,
                rightDepositArm,
                depositClaw
        );


        // init linear slides
        Motor extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_312);
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


        // set initial deposit positions
        verticalDeposit.setArmPosition(0.5);
        verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
        clipbot.setKlipperPosition(KlipperConstants.openPosition);

    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.addData("clipInventory", clipInventory);
        clipbot.update();
        telemetry.update();
    }

    private void initAdvanceMagazineMotion() {
        if (!inventoryStocked) return;
        if (clipInventory <= 0) {
            inventoryStocked = false;
            return;
        }


        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                clipbot.getMagazineFeederPosition()
        );

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );


        // Grab claw
        GrabVClaw grabSample = new GrabVClaw(0);

        // Stationary deposit arm
        LinearVArm lowerVArm = new LinearVArm(clawGrabWaitTime,
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Loosely hold sample
        MoveVClaw looselyHoldSample = new MoveVClaw(lowerVArm.getEndTime(), 0.1,
                new VClawState(VClawConstants.GRAB_POSITION),
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION)
        );

        // Advance feeder by one clip
        LinearMFeeder advanceFeeder = new LinearMFeeder(looselyHoldSample.getEndTime() + feederDelayTime,
                currentFeederPosition,
                targetFeederPosition
        );



        //// Plans
        MFeederPlan mFeederPlan;
        if (clipInventory==0) {
            telemetry.addData("Clip Inventory Is Zero!"," TRUE");
            LinearMFeeder resetFeeder = new LinearMFeeder(advanceFeeder.getEndTime(),
                    targetFeederPosition,
                    new MFeederState(0)
            );
            mFeederPlan = new MFeederPlan(clipbot,
                    advanceFeeder,
                    resetFeeder
            );
            inventoryStocked = false;
        } else {
            telemetry.addData("Clip Inventory Is Zero!"," FALSE");
            mFeederPlan = new MFeederPlan(clipbot,
                    advanceFeeder
            );
        }

        telemetry.update();


        // Klipper action
        MoveKlipper initKlipper = new MoveKlipper(0, KlipperConstants.openPosition);
        MoveKlipper klipSpecimen = new MoveKlipper(advanceFeeder.getEndTime(), KlipperConstants.closedPosition);
        MoveKlipper unklipSpecimen = new MoveKlipper(klipSpecimen.getEndTime()+klipperWaitTime, KlipperConstants.openPosition);

        KlipperPlan klipperPlan = new KlipperPlan(clipbot,
                initKlipper,
                klipSpecimen,
                unklipSpecimen
        );


        // Hold lift down
        LinearLift holdLiftDown = new LinearLift(0,
                new LiftState(holdLiftDownPosition),
                new LiftState(holdLiftDownPosition)
        );

        LiftPlan liftPlan = new LiftPlan(linearSlides,
                holdLiftDown
        );


        // Raise the arm
        LinearVArm raiseVArm = new LinearVArm(unklipSpecimen.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(0.5)
        );

        VArmPlan vArmPlan = new VArmPlan(verticalDeposit,
                lowerVArm,
                raiseVArm
        );



        // Drop specimen
        MoveVClaw releaseSample = new MoveVClaw(raiseVArm.getEndTime(), 0.01,
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION),
                new VClawState(VClawConstants.RELEASE_POSITION)
        );

        VClawPlan vClawPlan = new VClawPlan(verticalDeposit,
                grabSample,
                looselyHoldSample,
                releaseSample
        );


        // Synchronizer
        this.synchronizer = new Synchronizer(
                mFeederPlan,
                klipperPlan,
                vArmPlan,
                vClawPlan,
                liftPlan
        );
    }

}
