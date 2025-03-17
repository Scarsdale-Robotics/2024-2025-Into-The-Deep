package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.EducatedSearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.List;

@Disabled
public class AutoBlueObservation0Plus8 extends LinearOpMode {

    private AutonomousRobot robot;
    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private SampleDataBufferFilter sampleData;
    private ClipbotSubsystem clipbot;
    private VerticalDepositSubsystem verticalDeposit;

    private Synchronizer limelightAction;

    private int LOADED_CLIPS = 7;
    private int clipInventory = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(new Pose2d(-39, -63, new Rotation2d(Math.toRadians(0))));

        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,10), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(robot.limelightSubsystem, enableLimelight);
        limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();

        waitForStart();

        Synchronizer toSub = getToSubSync();
        toSub.start(); while (toSub.update());

        depositSampleGivenTranslationCorr();

        intakeSample();

        returnToObsWithExtendoMoveAndClipIntake();

        returnToSubWhileKlipping();

        while (clipInventory >= 0) {
            depositSampleGivenTranslationCorr();

            intakeSample();

            if (clipInventory <= 0) break;

            Synchronizer feederSync = SynchronizerAux.getFeederSync(--clipInventory, clipbot);
            feederSync.start(); while (feederSync.update());
        }
    }

    private void depositSampleGivenTranslationCorr() {
        Synchronizer liftLift = SynchronizerAux.getDepositExtend(linearSlides, verticalDeposit);
        liftLift.start(); while (liftLift.update());

        Synchronizer deposit = SynchronizerAux.getDepositAction(verticalDeposit, linearSlides);
        deposit.start(); while (deposit.update());
    }

    private void intakeSample() {
        double[] foundSample = waitForSamp();
        Synchronizer search;
        // init search
        search = new EducatedSearchMacro(
                foundSample,
                robot,
                1
        );
        search.start();
        sampleData.clearFilterData();
        boolean sampleMacroRunning = true;
        ExtendoState extendoVelocity = null;
        Synchronizer pickup = null;
        while (sampleMacroRunning) {
            // Search motion
            if (!sampleData.isFilterFull()) {
                boolean searchRunning = search.update();
                // Did not find sample
                if (!searchRunning) {
                    search.start();
                }
                // Try to detect sample
                else {
                    sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null
                    extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                }
            }
            // Pickup motion
            else {
                // Init pickup
                if ((pickup == null || !pickup.getIsRunning()) || search.getIsRunning()) {
                    search.stop();
                    pickup = SynchronizerAux.getPickupMotion(
                            extendoVelocity,
                            localization,
                            telemetry,
                            sampleData,
                            linearSlides,
                            horizontalIntake,
                            drive
                    );
                    pickup.start();
                }
                // Stop macro if macro ended
                if (pickup.getIsRunning() && !pickup.update()) {
                    pickup.stop();
                    sampleMacroRunning = false;
                    // init extendo retract macro
                    Synchronizer extendoRetract = new ExtendoRetractMacro(linearSlides);
                    extendoRetract.start();
                    while (extendoRetract.update());
                }
            }
        }
    }

    // prepares to intake clips
    private Synchronizer getToSubSync() {
        // translation Plan
        LinearTranslation toSub = new LinearTranslation(0,
                new TranslationState(-9, 72-9),
                new TranslationState(-2, 72-48+9)
        );

        TranslationPlan translationPlan = new TranslationPlan(
                drive, localization,
                toSub
        );

        // put all the Plans into a Synchronizer
        return new Synchronizer(
                translationPlan
        );
    }

    private void returnToSubWhileKlipping() {
        Synchronizer backToSub = getBackToSub();
        backToSub.start(); while (backToSub.update());

        Synchronizer feederSync = SynchronizerAux.getFeederSync(--clipInventory, clipbot);
        feederSync.start(); while (feederSync.update());

        boolean backToSubRunning = true,
                feederSyncRunning = true;

        int syncsDone = 0;
        while (syncsDone < 2) {
            if (backToSubRunning) {
                backToSubRunning = backToSub.update();
                if (!backToSubRunning) {
                    syncsDone++;
                }
            }
            if (feederSyncRunning) {
                feederSyncRunning = feederSync.update();
                if (!feederSyncRunning) {
                    syncsDone++;
                }
            }
        }
    }

    private Synchronizer getBackToSub() {
        LinearTranslation toSub = new LinearTranslation(0,
                new TranslationState(localization.getPose()),
                new TranslationState(-2, 72-48+9)
        );

        TranslationPlan translationPlan = new TranslationPlan(
                drive, localization,
                toSub
        );

        return new Synchronizer(translationPlan);
    }

    private void returnToObsWithExtendoMoveAndClipIntake() {
        Synchronizer extendoRetract = new ExtendoRetractMacro(linearSlides);
        Synchronizer toObs = getToObsSync();
        extendoRetract.start();
        boolean extendoRetractRunning = true;
        boolean toObsRunning = false;
        int syncsDone = 0;
        while (syncsDone < 2) {
            if (extendoRetract.getElapsedTime() > 1) {  // seconds?
                toObs.start();
                toObsRunning = true;
            }

            if (extendoRetractRunning) {
                extendoRetractRunning = extendoRetract.update();
                if (!extendoRetractRunning) {
                    syncsDone++;
                }
            }
            if (toObsRunning) {
                toObsRunning = toObs.update();
                if (!toObsRunning) {
                    syncsDone++;
                }
            }
        }
        Synchronizer clipIntake = SynchronizerAux.getClipIntakeSync(clipbot, localization, drive);
        clipIntake.start(); while (clipIntake.update());
        clipInventory = LOADED_CLIPS;
    }

    private Synchronizer getToObsSync() {
        LinearTranslation toClips = new LinearTranslation(0,
                new TranslationState(-2, 72-48+9),
                new TranslationState(-36, 72-9)
        );

        TranslationPlan translationPlan = new TranslationPlan(
                drive, localization,
                toClips
        );

        return new Synchronizer(translationPlan);
    }

    private Synchronizer getShiftSync(int trans) {
        TranslationState currentPosition = new TranslationState(localization.getPose());
        LinearTranslation shift = new LinearTranslation(0,
                currentPosition,
                currentPosition.plus(new TranslationState(trans, 0))
        );
        TranslationPlan translationPlan = new TranslationPlan(
                drive, localization,
                shift
        );
        return new Synchronizer(translationPlan);
    }

    private double[] waitForSamp() {
        double[] foundSample = waitForSamp(5);
        if (foundSample!=null) return foundSample;

        for (int i=0;true;i++) {
            Synchronizer sync = getShiftSync((((i+1)/2)%2==0)?5:-4);
            sync.start(); while (sync.update());
            foundSample = waitForSamp(5);
            if (foundSample!=null) return foundSample;
        }
    }

    private double[] waitForSamp(int runsLeft) {
        if (runsLeft <= 0) return null;

        // move at end
        limelightAction.update();
        robot.update();

        // Look for limelight samples
        Pose2d botPose = localization.getPose();
        List<double[]> samplePositions;
        if (robot.teamColor==AutonomousRobot.TeamColor.BLUE) {
            samplePositions = robot.limelightSubsystem.getBlueSamplePositions();
        } else {
            samplePositions = robot.limelightSubsystem.getRedSamplePositions();
        }
        double[] foundSample = null;
        if (!samplePositions.isEmpty()) {
            double closestDistance = Double.MAX_VALUE;
            double[] closestSample = null;
            for (double[] samplePosition : samplePositions) {
                double distance = Math.hypot(samplePosition[0]-botPose.getX(), samplePosition[1]-botPose.getY());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestSample = samplePosition;
                }
            }
            foundSample = closestSample;
        }

        if (foundSample==null) {
            waitForSamp(runsLeft - 1);
        }
        return foundSample;
    }

    private void initialize(Pose2d initialPose) {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                initialPose,
                AutonomousRobot.TeamColor.BLUE,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
        this.telemetry = robot.telemetry;
        this.horizontalIntake = robot.horizontalIntake;
        this.overheadCamera = robot.overheadCamera;
        this.linearSlides = robot.linearSlides;
        this.localization = robot.localization;
        this.drive = robot.drive;
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -2;
        ExtendoConstants.MAX_MOTOR_VELOCITY = 80;

        this.sampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                0.04375,
                1,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
        robot.setOverheadSampleDataBufferFilter(this.sampleData);
        SampleDataBufferFilter.FILTER_ERROR_TOLERANCE = 0.5;

        // init servos
        horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
        horizontalIntake.setWristAngle(Math.PI/2);
        horizontalIntake.setArmPosition(0.25);
    }
}
