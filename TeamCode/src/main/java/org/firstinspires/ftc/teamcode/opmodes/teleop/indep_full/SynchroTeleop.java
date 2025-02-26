package org.firstinspires.ftc.teamcode.opmodes.teleop.indep_full;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initPinpoint;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;
import static org.firstinspires.ftc.teamcode.Auxiliary.initWebcam;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakePlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.MoveMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.function.Supplier;

@TeleOp(name = "SynchroTeleop")
public class SynchroTeleop extends LinearOpMode {
    private boolean lastAutoIntakeCommand = false;

    private Synchronizer autoIntake;

    private Synchronizer[] manualIntakeSynchronizer;
    private Synchronizer clipIntakeSynchronizer;

    private double wristPos = 0;
    private double extendoPos = 0;

    private static final int MAX_WRIST = 1;
    private static final int MIN_WRIST = 0;

    private static final double EXTENDO_SPEED = 3.0;

    private static final int MAX_EXTENDO = 2000;
    private static final int MIN_EXTENDO = 200;  // pos must be safe to flip arm

    private static final double TRANSFER_ARM = 0.2;
    private static final double TRANSFER_WRIST = 0;
    private static final double APPROACH_ARM = 0.95;
    private static final double INTAKE_ARM = 1.05;
    private static final int TRANSFER_EXTENDO = 200;

    private static final double M_INTAKE_OPEN = 0;
    private static final double M_INTAKE_UP = 0;
    private static final double M_INTAKE_CLOSED = 0;
    private static final double M_LOADER_OPEN = 0;
    private static final double M_LOADER_CLOSED = 0;
    private static final int M_FEEDER_MAX = 10000;
    private static final int M_FEEDER_MIN = 0;

    private static boolean RUN_INTAKE = true;
    private static boolean RUN_CLIPBOT = true;

    private LinearSlidesSubsystem linearSlides;
    private HorizontalIntakeSubsystem horizontalIntake;
    private ClipbotSubsystem clipbot;
    private OverheadCameraSubsystem overheadCamera;
    private DriveSubsystem drive;
    private LocalizationSubsystem localization;
    private GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean autoIntakingEnabled = true;
        int intakeIndex = 0;

//        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");

        // NO MOVEMENT BEFORE START

        // TODO: check motor types are correct
        linearSlides = new LinearSlidesSubsystem(
                initMotor(hardwareMap, "extendo", Motor.GoBILDA.RPM_312),
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
                initServo(hardwareMap, "magazineLoader1"),
                initServo(hardwareMap, "magazineLoader2"),
                initServo(hardwareMap, "klipper"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_312),
                telemetry
        );

        overheadCamera = new OverheadCameraSubsystem(
                initWebcam(hardwareMap, "Webcam 1"),
                telemetry
        );

        pinpoint = initPinpoint(
                hardwareMap,
                "pinpoint",
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        localization = new LocalizationSubsystem(
                new Pose2d(0,0,new Rotation2d(Math.toRadians(90))),
                pinpoint,
                this,
                telemetry
        );

        drive = new DriveSubsystem(
                initMotor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                initMotor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                initMotor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312),
                initMotor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312)
        );

        waitForStart();

        runtime = new ElapsedTime(0);
        runtime.reset();

        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.5);

        boolean autoIntakeRunning = false;
        boolean manualIntakeRunning = false;
        boolean clipIntakeRunning = false;
        while (opModeIsActive()) {
            ///////////////////
            // SAMPLE INTAKE //
            ///////////////////
            if (RUN_INTAKE) {
                boolean currAutoIntakeCommand = gamepad1.dpad_left && gamepad1.cross;
                if (lastAutoIntakeCommand && !currAutoIntakeCommand) {
                    autoIntakingEnabled = !autoIntakingEnabled;
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                }
                lastAutoIntakeCommand = currAutoIntakeCommand;

                if (autoIntakingEnabled) {
                    if (!autoIntakeRunning) {
                        // updateAutoIntakeSynchronizer();
                    }

                    if (gamepad1.triangle) {
                        autoIntake.start();

                        // set extendoPos and wristPos here

                        autoIntakeRunning = true;
                        manualIntakeRunning = false;
                    }
                } else {
                    if (!manualIntakeRunning) {
                        updateIntakeSynchronizers();
                    }

                    // circle --> try next action
                    // triangle --> retry current action
                    // square --> try last action

                    if (gamepad1.square) intakeIndex--;
                    if (gamepad1.circle) intakeIndex++;

                    intakeIndex %= manualIntakeSynchronizer.length;

                    if (gamepad1.square || gamepad1.triangle || gamepad1.circle) {
                        manualIntakeSynchronizer[intakeIndex].start();

                        manualIntakeRunning = true;
                        autoIntakeRunning = false;
                    }

                    if (gamepad2.left_stick_button)
                        wristPos = (Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) + Math.PI) % Math.PI / Math.PI * (MAX_WRIST - MIN_WRIST) + MIN_WRIST;
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
                    manualIntakeRunning = manualIntakeSynchronizer[intakeIndex].update();
                }
            }

            if (RUN_CLIPBOT) {
                updateClipbotSynchronizers();

                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    clipIntakeSynchronizer.start();
                    clipIntakeRunning = true;
                }

                if (clipIntakeRunning) {
                    clipIntakeRunning = clipIntakeSynchronizer.update();
                }
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

        manualIntakeSynchronizer = new Synchronizer[]{
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

    private void updateClipbotSynchronizers() {
        Supplier<Synchronizer> clipIntake = () -> {
            MoveMIntake intakeOpen = new MoveMIntake(0, M_INTAKE_OPEN);
            MoveMLoader loaderOpen = new MoveMLoader(0, M_LOADER_OPEN);

            double INTAKE_SHIFT = 32;
            double INTAKE_BACK = 12;
            LinearTranslation shiftForIntake = new LinearTranslation(
                    new TimeSpan(
                            Math.max(
                                    intakeOpen.getEndTime(),
                                    loaderOpen.getEndTime()
                            ),
                            Math.max(
                                    intakeOpen.getEndTime(),
                                    loaderOpen.getEndTime()
                            ) + 10
                    ),
                    getCurrentTranslation(),
                    getCurrentTranslation().plus(
                            new TranslationState(
                                    INTAKE_SHIFT,
                                    localization.getH() + Math.PI / 2,
                                    true
                            )
                    )
            );

            MoveMIntake intakeUp = new MoveMIntake(shiftForIntake.getEndTime(),
                    M_INTAKE_UP
            );
            LinearTranslation backup = new LinearTranslation(
                    new TimeSpan(
                            intakeUp.getEndTime(),
                            intakeUp.getEndTime() + 5
                    ),
                    getCurrentTranslation(),
                    getCurrentTranslation().plus(
                            new TranslationState(
                                    INTAKE_BACK,
                                    localization.getH() + Math.PI,
                                    true
                            )
                    )
            );
            MoveMIntake intakeClose = new MoveMIntake(
                    backup.getEndTime(),
                    M_INTAKE_CLOSED
            );
            MoveMLoader loaderClose = new MoveMLoader(
                    intakeClose.getEndTime(),
                    M_LOADER_CLOSED
            );

            MIntakePlan intakePlan = new MIntakePlan(
                    clipbot,
                    intakeOpen, intakeUp, intakeClose
            );
            MLoaderPlan loaderPlan = new MLoaderPlan(
                    clipbot,
                    loaderOpen, loaderClose
            );
            TranslationPlan translationPlan = new TranslationPlan(
                    drive, localization,
                    shiftForIntake, backup
            );

            return new Synchronizer(
                    intakePlan,
                    loaderPlan,
                    translationPlan
            );
        };

        Supplier<Synchronizer> TEST_NO_TRANSLATION_clipIntake = () -> {
            MoveMIntake intakeOpen = new MoveMIntake(0, M_INTAKE_OPEN);
            MoveMLoader loaderOpen = new MoveMLoader(0, M_LOADER_OPEN);

            MoveMIntake intakeUp = new MoveMIntake(
                    Math.max(
                            intakeOpen.getEndTime(),
                            loaderOpen.getEndTime()
                    ) + 5,
                    M_INTAKE_UP
            );

            MoveMIntake intakeClose = new MoveMIntake(
                    intakeUp.getEndTime() + 5,
                    M_INTAKE_CLOSED
            );
            MoveMLoader loaderClose = new MoveMLoader(
                    intakeClose.getEndTime(),
                    M_LOADER_CLOSED
            );

            MIntakePlan intakePlan = new MIntakePlan(
                    clipbot,
                    intakeOpen, intakeUp, intakeClose
            );
            MLoaderPlan loaderPlan = new MLoaderPlan(
                    clipbot,
                    loaderOpen, loaderClose
            );

            return new Synchronizer(
                    intakePlan,
                    loaderPlan
            );
        };

        clipIntakeSynchronizer = clipIntake.get();
    }

    private TranslationState getCurrentTranslation() {
        return new TranslationState(localization.getX(), localization.getY());
    }
}
