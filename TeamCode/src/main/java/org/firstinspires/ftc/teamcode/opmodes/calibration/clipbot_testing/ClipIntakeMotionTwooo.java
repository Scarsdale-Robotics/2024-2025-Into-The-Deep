package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperState;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements.MoveKlipper;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakePlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.LinearMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.MoveMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;

import java.util.function.Supplier;

@Config
@Autonomous(name="Clip Intake Motion TWOOO", group = "Calibration")
public class ClipIntakeMotionTwooo extends LinearOpMode {

    private boolean lastAutoIntakeCommand = false;

    private Synchronizer autoIntake;

    private Synchronizer[] manualIntakeSynchronizer;
    private Synchronizer clipIntakeSynchronizer;
    private Synchronizer feedOneClipSynchronizer;
    private Synchronizer feederResetSynchronizer;
    private Synchronizer openKlipperSynchronizer;
    private Synchronizer closeKlipperSynchronizer;

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

        // NO MOVEMENT BEFORE START

        clipbot = new ClipbotSubsystem(
                initServo(hardwareMap, "magazineIntake"),
                initServo(hardwareMap, "magazineLoaderClose"),
                initServo(hardwareMap, "magazineLoaderFar"),
                initServo(hardwareMap, "klipper"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_312),
                telemetry
        );

        waitForStart();

        runtime = new ElapsedTime(0);
        runtime.reset();

        boolean lastKlipperPressed = false;
        boolean lastIntakePressed = false;
        boolean lastFeederPressed = false;

        boolean klipperOpen = true;
        boolean clipIntakeRunning = false;
        boolean feederRunning = false;
        boolean klipperRunning = false;
        while (opModeIsActive()) {
            if (!clipIntakeRunning) {
                updateIntakeSyncs();
            }
            boolean currIntakePressed = gamepad1.left_bumper && gamepad1.right_bumper;
            if (!currIntakePressed && lastIntakePressed) {
                clipIntakeSynchronizer.start();
                clipIntakeRunning = true;
            }
            if (clipIntakeRunning) {
                clipIntakeRunning = clipIntakeSynchronizer.update();
            }
            lastIntakePressed = currIntakePressed;

            if (!feederRunning) {
                updateFeederSyncs();
            }
            boolean currFeederPressed = gamepad1.cross;
            if (!currFeederPressed && lastFeederPressed) {
                feedOneClipSynchronizer.start();
                feederRunning = true;
            }
            if (feederRunning) {
                feederRunning = feedOneClipSynchronizer.update();
            }
            lastFeederPressed = currFeederPressed;

            if (!klipperRunning) {
                updateKlipperSyncs();
            }
            boolean currKlipperPressed = gamepad1.triangle;
            if (!currKlipperPressed && lastKlipperPressed) {
                klipperOpen = !klipperOpen;
                if (klipperOpen) openKlipperSynchronizer.start();
                else closeKlipperSynchronizer.start();
                klipperRunning = true;
            }
            if (klipperRunning) {
                if (klipperOpen) klipperRunning = openKlipperSynchronizer.update();
                else klipperRunning = closeKlipperSynchronizer.update();
            }
            lastKlipperPressed = currKlipperPressed;
        }
    }

    private void updateIntakeSyncs() {
        Supplier<Synchronizer> TEST_NO_TRANSLATION_clipIntake = () -> {
            MoveMIntake intakeOpen = new MoveMIntake(0, MIntakeConstants.openPosition);
            MoveMLoader loaderOpen = new MoveMLoader(0, MLoaderConstants.openPosition);

            MoveMIntake intakeUp = new MoveMIntake(
                    Math.max(
                            intakeOpen.getEndTime(),
                            loaderOpen.getEndTime()
                    ) + 5,
                    MIntakeConstants.upPosition
            );

            LinearMIntake intakeClose = new LinearMIntake(
                    new TimeSpan(
                            intakeUp.getEndTime() + 5,
                            intakeUp.getEndTime() + 10
                    ),
                    new MIntakeState(MIntakeConstants.upPosition),
                    new MIntakeState(MIntakeConstants.closedPosition)
            );

            MoveMLoader loaderClose = new MoveMLoader(
                    intakeClose.getEndTime(),
                    MLoaderConstants.maxClosedPosition
            );

            MoveMLoader loaderRelease = new MoveMLoader(
                    loaderClose.getEndTime() + 1,
                    MLoaderConstants.partialClosedPosition
            );

            MoveMLoader loaderClose2 = new MoveMLoader(
                    loaderRelease.getEndTime()+0.5,
                    MLoaderConstants.maxClosedPosition
            );

            MoveMLoader loaderRelease2 = new MoveMLoader(
                    loaderClose2.getEndTime()+1,
                    MLoaderConstants.partialClosedPosition
            );

            MIntakePlan intakePlan = new MIntakePlan(
                    clipbot,
                    intakeOpen, intakeUp, intakeClose
            );
            MLoaderPlan loaderPlan = new MLoaderPlan(
                    clipbot,
                    loaderOpen, loaderClose, loaderRelease, loaderClose2, loaderRelease2
            );

            return new Synchronizer(
                    intakePlan,
                    loaderPlan
            );
        };

        clipIntakeSynchronizer = TEST_NO_TRANSLATION_clipIntake.get();
    }

    private void updateFeederSyncs() {
        Supplier<Synchronizer> feedOneClip = () -> {
            MFeederState currentPos = new MFeederState(clipbot.getMagazineFeederPosition());
            LinearMFeeder shiftFeeder = new LinearMFeeder(
                    0,
                    currentPos,
                    currentPos.plus(new MFeederState(MFeederConstants.INCHES_PER_CLIP))
            );

            MFeederPlan feederPlan = new MFeederPlan(
                    clipbot,
                    shiftFeeder
            );

            return new Synchronizer(
                    feederPlan
            );
        };

        feedOneClipSynchronizer = feedOneClip.get();
    }

    private void updateKlipperSyncs() {
        Supplier<Synchronizer> closeKlipper = () -> {
            MoveKlipper klipperClose = new MoveKlipper(0, KlipperConstants.closedPosition);
            KlipperPlan klipperPlan = new KlipperPlan(
                    clipbot,
                    klipperClose
            );

            return new Synchronizer(klipperPlan);
        };

        Supplier<Synchronizer> openKlipper = () -> {
            MoveKlipper klipperOpen = new MoveKlipper(0, KlipperConstants.openPosition);
            KlipperPlan klipperPlan = new KlipperPlan(
                    clipbot,
                    klipperOpen
            );

            return new Synchronizer(klipperPlan);
        };

        closeKlipperSynchronizer = closeKlipper.get();
        openKlipperSynchronizer = openKlipper.get();
    }


}
