package org.firstinspires.ftc.teamcode.opmodes.teleop.indep_full;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;
import static org.firstinspires.ftc.teamcode.Auxiliary.initWebcam;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.GrabHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;

import java.util.function.Supplier;

public class SynchroTeleop extends LinearOpMode {
    private boolean lastAutoIntakeCommand = false;

    private Synchronizer autoIntake;

    private Synchronizer[] manualIntakes;
    private Synchronizer[] magazineSyncs;

    private double wristPos = 0;
    private double extendoPos = 0;

    private static final int MAX_WRIST = 1;
    private static final int MIN_WRIST = 0;

    private static final double EXTENDO_SPEED = 3.0;

    private static final int MAX_EXTENDO = 2000;
    private static final int MIN_EXTENDO = 200;  // pos must be safe to flip arm

    private static final int MAX_MAG_PUSHER = 10000;
    private static final int MIN_MAG_PUSHER = 0;

    private static final double TRANSFER_ARM = 0.2;
    private static final double TRANSFER_WRIST = 0;
    private static final double APPROACH_ARM = 0.95;
    private static final double INTAKE_ARM = 1.05;
    private static final int TRANSFER_EXTENDO = 200;

    private LinearSlidesSubsystem linearSlides;
    private HorizontalIntakeSubsystem horizontalIntake;
    private ClipbotSubsystem clipbot;
    private OverheadCameraSubsystem overheadCamera;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean autoIntakingEnabled = true;
        int intakeIndex = 0;

//        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");

        // NO MOVEMENT BEFORE START

        // TODO: check motor types are correct
        linearSlides = new LinearSlidesSubsystem(
                initMotor(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620),
                initMotor(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312),
                initMotor(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312),
                telemetry
        );

        horizontalIntake = new HorizontalIntakeSubsystem(
                initServo(hardwareMap,"leftHorizontalArm"),
                initServo(hardwareMap, "rightHorizontalArm"),
                initServo(hardwareMap, "horizontalWrist"),
                initServo(hardwareMap, "horizontalClaw")
        );

        clipbot = new ClipbotSubsystem(
                initServo(hardwareMap, "magazineIntake"),
                initServo(hardwareMap, "magazineLoader"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_312),
                telemetry
        );

        overheadCamera = new OverheadCameraSubsystem(
                initWebcam(hardwareMap, "Webcam 1"),
                telemetry
        );

//        GoBildaPinpointDriver pinpoint = initPinpoint(
//                hardwareMap,
//                "pinpoint",
//                GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD
//        );



        waitForStart();

        runtime = new ElapsedTime(0);
        runtime.reset();

        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.5);

        clipbot.homeMagazineFeeder(this, telemetry);  // TODO: TEST IF HOMING HERE IS HELPFUL

        boolean autoIntakeRunning = false;
        boolean manualIntakeRunning = false;
        while (opModeIsActive()) {
            updateIntakeSynchronizers();

            boolean currAutoIntakeCommand = gamepad1.dpad_left && gamepad1.cross;
            if (lastAutoIntakeCommand && !currAutoIntakeCommand) {
                autoIntakingEnabled = !autoIntakingEnabled;
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }
            lastAutoIntakeCommand = currAutoIntakeCommand;

            if (autoIntakingEnabled) {
                if (gamepad1.triangle) {
                    autoIntake.start();

                    // set extendoPos and wristPos here

                    autoIntakeRunning = true;
                    manualIntakeRunning = false;
                }
            } else {
                // circle --> try next action
                // triangle --> retry current action
                // square --> try last action

                if (gamepad1.square) intakeIndex--;
                if (gamepad1.circle) intakeIndex++;

                intakeIndex %= manualIntakes.length;

                if (gamepad1.square || gamepad1.triangle || gamepad1.circle) {
                    manualIntakes[intakeIndex].start();

                    manualIntakeRunning = true;
                    autoIntakeRunning = false;
                }

                if (gamepad2.left_stick_button) wristPos = (Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) + Math.PI) % Math.PI / Math.PI * (MAX_WRIST - MIN_WRIST) + MIN_WRIST;
                extendoPos += (gamepad1.right_trigger - gamepad1.left_trigger) * EXTENDO_SPEED;

//                if (intakeIndex == 0 || intakeIndex == 1) {
//                    double dth = 0.01;
//                    if (gamepad1.dpad_left) wristPos -= dth;
//                    if (gamepad1.dpad_right) wristPos += dth;
//                }

                extendoPos = Math.max(Math.min(extendoPos, MAX_EXTENDO), MIN_EXTENDO);
                wristPos = Math.max(Math.min(wristPos, MAX_WRIST), MIN_WRIST);

//                Servo horizontalClaw = hardwareMap.get(ServoImplEx.class, "horizontalClaw");
            }

            if (autoIntakeRunning) {
                autoIntakeRunning = autoIntake.update();
            } else if (manualIntakeRunning) {
                manualIntakeRunning = manualIntakes[intakeIndex].update();
            }
        }
    }

    private void updateIntakeSynchronizers() {
        autoIntake = new Synchronizer();

        Supplier<Synchronizer> transferToApproachSync = () -> {
            LinearHArm armDown = new LinearHArm(0,
                    new HArmState(TRANSFER_ARM),
                    new HArmState(APPROACH_ARM)
            );
            MoveHWrist wristFinal = new MoveHWrist(0, wristPos);
            LinearExtendo extendoPrime = new LinearExtendo(0,
                    new ExtendoState(TRANSFER_EXTENDO),
                    new ExtendoState(extendoPos)
            );
            ReleaseHClaw clawRelease = new ReleaseHClaw(0);

            HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                    wristFinal
            );
            HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                    armDown
            );
            HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                    clawRelease
            );
            ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides, extendoPrime);

            return new Synchronizer(
                    extendoPlan,
                    h_claw_plan,
                    h_arm_plan,
                    h_wrist_plan
            );
        };

        Supplier<Synchronizer> intakeSync = () -> {
            LinearHArm armDown = new LinearHArm(0,
                    new HArmState(APPROACH_ARM),
                    new HArmState(INTAKE_ARM)
            );
            MoveHWrist wristFinal = new MoveHWrist(0, wristPos);
            ReleaseHClaw clawRelease = new ReleaseHClaw(0);
            GrabHClaw clawGrab = new GrabHClaw(armDown.getEndTime());
            LinearHArm armUp = new LinearHArm(clawGrab.getEndTime(),
                    new HArmState(INTAKE_ARM),
                    new HArmState(APPROACH_ARM)
            );

            HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                    wristFinal
            );
            HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                    armDown,
                    armUp
            );
            HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                    clawRelease,
                    clawGrab
            );

            return new Synchronizer(
                    h_claw_plan,
                    h_arm_plan,
                    h_wrist_plan
            );
        };

        Supplier<Synchronizer> approachToTransferSync = () -> {
            LinearHArm armUp = new LinearHArm(0,
                    new HArmState(APPROACH_ARM),
                    new HArmState(TRANSFER_ARM)
            );
            MoveHWrist wristFinal = new MoveHWrist(0, TRANSFER_WRIST);
            LinearExtendo extendoRetract = new LinearExtendo(0,
                    new ExtendoState(extendoPos),
                    new ExtendoState(TRANSFER_EXTENDO)
            );
            ReleaseHClaw clawRelease = new ReleaseHClaw(
                    Math.max(
                            extendoRetract.getEndTime(),
                            armUp.getEndTime()
                    )
            );

            HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                    wristFinal
            );
            HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                    armUp
            );
            HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                    clawRelease
            );
            ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides, extendoRetract);

            return new Synchronizer(
                    extendoPlan,
                    h_claw_plan,
                    h_arm_plan,
                    h_wrist_plan
            );
        };

        manualIntakes = new Synchronizer[]{
            transferToApproachSync.get(),
            intakeSync.get(),
            approachToTransferSync.get()
        };


        // TODO: Implement autoIntake synchronizer
//        LinearExtendo extendo1 = new LinearExtendo(0,
//                new ExtendoState(0),
//                new ExtendoState(12)
//        );
//        ExtendoPlan extendoPlan = new ExtendoPlan(null,
//                extendo1
//        );
//        this.synchronizer = new Synchronizer(extendoPlan);
    }

    private void updateMagazineSynchronizers() {
//        Supplier<Synchronizer> magazineOpen = () -> {
//
//            return new Synchronizer();
//        };
//
//        Supplier<Synchronizer> magazineClose = () -> {
//            // slow
//        };
    }
}
