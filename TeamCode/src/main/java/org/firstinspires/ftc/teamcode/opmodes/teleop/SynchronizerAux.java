package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.GrabHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements.MoveKlipper;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

import java.util.function.Supplier;

@Config
public class SynchronizerAux {
    public static double INTAKE_SHIFT = 24;
    public static double INTAKE_BACK = 12;
    public static Synchronizer getClipIntakeSync(
            ClipbotSubsystem clipbot,
            LocalizationSubsystem localization,
            DriveSubsystem drive
    ) {
        MoveMIntake intakeOpen = new MoveMIntake(0, MIntakeConstants.openPosition);
        MoveMLoader loaderOpen = new MoveMLoader(0, MLoaderConstants.openPosition);

//        double initEnd = Math.max(intakeOpen.getEndTime(), loaderOpen.getEndTime());
        LinearTranslation wallShift = new LinearTranslation(
                Math.max(intakeOpen.getEndTime(), loaderOpen.getEndTime()),
                getCurrentTranslation(localization),
                getCurrentTranslation(localization).plus(
                        new TranslationState(
                                INTAKE_SHIFT,
                                localization.getH() + Math.PI / 2,
                                true
                        )
                )
        );

        MoveMIntake intakeUp = new MoveMIntake(
                wallShift.getEndTime(),
                MIntakeConstants.upPosition
        );

        LinearTranslation backup = new LinearTranslation(
                intakeUp.getEndTime(),
                getCurrentTranslation(localization),
                getCurrentTranslation(localization).plus(
                        new TranslationState(
                                INTAKE_SHIFT,
                                localization.getH() + Math.PI / 2,
                                true
                        )
                )
        );

        LinearMIntake intakeClose = new LinearMIntake(
                new TimeSpan(
                        backup.getEndTime(), backup.getEndTime() + 3
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

        MIntakePlan intakePlan = new MIntakePlan(
                clipbot,
                intakeOpen, intakeUp, intakeClose
        );
        MLoaderPlan loaderPlan = new MLoaderPlan(
                clipbot,
                loaderOpen, loaderClose, loaderRelease
        );

        TranslationPlan translationPlan = new TranslationPlan(
                drive, localization,
                wallShift, backup
        );

        return new Synchronizer(
                intakePlan,
                loaderPlan,
                translationPlan
        );
    }

    public static Synchronizer getPickupMotion(
            ExtendoState extendoVelocity,
            LocalizationSubsystem localization,
            Telemetry telemetry,
            SampleDataBufferFilter sampleData,
            LinearSlidesSubsystem linearSlides,
            HorizontalIntakeSubsystem horizontalIntake,
            DriveSubsystem drive
    ) {
            // Unpack bot pose
            Pose2d botPose = localization.getPose();
            double x_bot = botPose.getX();
            double y_bot = botPose.getY();
            double heading_bot = botPose.getHeading();

            // Unpack sample pose
            Pose2d samplePose = sampleData.getFilteredSamplePosition(telemetry);
            double x_sample = samplePose.getX();
            double y_sample = samplePose.getY();
            double theta_sample = samplePose.getHeading();

            // Get extendo state
            double x_extendo = linearSlides.getExtendoPosition();
            ExtendoState extendoPosition = new ExtendoState(x_extendo);
            double x_extendo_min = OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0];

            // Get subsystem setpoints
            TranslationState translationTarget = new TranslationState(
                    x_sample,
                    y_bot + Math.min(0, (y_sample-y_bot) - x_extendo_min)
            );
            RotationState rotationTarget = new RotationState(
                    heading_bot + normalizeAngle(Math.PI/2-heading_bot)
            );
            ExtendoState extendoTarget = new ExtendoState(
                    Math.max(0, (y_sample-y_bot) - x_extendo_min)
            );
            double hWristTarget = normalizeAngle(theta_sample);

            telemetry.addData("[DEBUG] botPose.getHeading()", botPose.getHeading());
            telemetry.addData("[DEBUG] rotationTarget.getHeading()", rotationTarget.getHeading());
            telemetry.addData("[DEBUG] extendoTarget.getLength()", extendoTarget.getLength());
            telemetry.addData("[DEBUG] hWristTarget", hWristTarget);

            telemetry.update();

            //// SYNCHRONIZER
            // Extendo
            LinearTranslation translation = new LinearTranslation(0,
                    new TranslationState(botPose),
                    translationTarget
            );

            LinearRotation rotation = new LinearRotation(0,
                    new RotationState(botPose),
                    rotationTarget
            );

//        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
//        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / 3;
            DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                    extendoPosition,
                    extendoTarget,
                    extendoVelocity
            );
//        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

            double intakeDelay = 0.25;

            // Move arm down
            LinearHArm h_arm_down = new LinearHArm(intakeDelay+Math.max(Math.max(extendoOut.getEndTime(), rotation.getEndTime()), translation.getEndTime()),
                    new HArmState(0.9),
                    new HArmState(HArmConstants.armLeftDownPosition),
                    true
            );
            MoveHWrist h_wrist_align = new MoveHWrist(extendoOut.getStartTime(), hWristTarget);

            // Pick up and move arm up
            GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime(), true);
            LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime(),
                    new HArmState(HArmConstants.armLeftDownPosition),
                    new HArmState(0.9)
            );
            MoveHWrist h_wrist_reset = new MoveHWrist(h_arm_up.getEndTime(), 0, true);

            // Retract extendo
            LinearExtendo extendoIn = new LinearExtendo(h_wrist_reset.getStartTime(),
                    extendoTarget,
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset)
            );

            // Create Plans
            TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                    translation
            );
            RotationPlan rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );
            ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                    extendoOut,
                    extendoIn
            );
            HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                    h_wrist_align,
                    h_wrist_reset
            );
            HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                    h_arm_down,
                    h_arm_up
            );
            HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                    h_claw_grab
            );

            // Synchronizer
            return new Synchronizer(
                    translationPlan,
                    rotationPlan,
                    extendo_plan,
                    h_arm_plan,
                    h_wrist_plan,
                    h_claw_plan
            );
    }

    public static Synchronizer getDepositAction(
        VerticalDepositSubsystem verticalDeposit,
        LinearSlidesSubsystem linearSlides
    ) {
        LinearVArm tiltArm = new LinearVArm(0,
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift lowerLift = new LinearLift(tiltArm.getEndTime(),
                new LiftState(LiftConstants.depositPosition),
                new LiftState(LiftConstants.transferPosition)
        );

        LinearVArm endArm = new LinearVArm(tiltArm.getEndTime() + 1,
                new VArmState(VArmConstants.armLeftDepositPosition),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // DO NOT open claw
        VArmPlan vArmPlan = new VArmPlan(verticalDeposit,
                tiltArm,
                endArm
        );

        LiftPlan liftPlan = new LiftPlan(linearSlides,
                lowerLift
        );

        return new Synchronizer(
                vArmPlan,
                liftPlan
        );
    }

    public static Synchronizer getDepositExtend(
        LinearSlidesSubsystem linearSlides,
        VerticalDepositSubsystem verticalDeposit
    ) {
        LinearVArm armBack = new LinearVArm(0,
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftPreDepositPosition)
        );

        LinearLift liftUp = new LinearLift(armBack.getEndTime(),
                new LiftState(LiftConstants.specMakerPosition),
                new LiftState(LiftConstants.depositPosition)
        );

        VArmPlan vArmPlan = new VArmPlan(verticalDeposit,
                armBack
        );

        LiftPlan liftPlan = new LiftPlan(linearSlides,
                liftUp
        );

        return new Synchronizer(
                vArmPlan,
                liftPlan
        );
    }

    public static Synchronizer getTransferSync(
        LinearSlidesSubsystem linearSlides,
        HorizontalIntakeSubsystem horizontalIntake,
        VerticalDepositSubsystem verticalDeposit
    ) {
        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(0,
                new ExtendoState(linearSlides.getExtendoPosition()),
                new ExtendoState(4.3)
        );

        LinearLift liftUp = new LinearLift(0,
                new LiftState(linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        ReleaseVClaw releaseVClaw = new ReleaseVClaw(0);

        // Horizontal arm goes up
        LinearHArm hArmUp = new LinearHArm(Math.max(extendoIn.getEndTime(), liftUp.getEndTime()),
                new HArmState(0.9),
                new HArmState(0.48)
        );

        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(hArmUp.getEndTime(),
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Deposit claw grabs sample
        GrabVClaw grabVClaw = new GrabVClaw(vArmDown.getEndTime() + 0.25);

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getEndTime());

        // Deposit arm moves out of the way
        LinearVArm upVArm = new LinearVArm(releaseHClaw.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(upVArm.getEndTime(),
                new HArmState(0.48),
                new HArmState(0.9)
        );

        // Deposit arm prepares for clip zone
        LinearVArm toClipperVArm = new LinearVArm(hArmDown.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Lift approaches clip zone
        LinearLift toClipperLift = new LinearLift(hArmDown.getEndTime(),
                new LiftState(linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.specMakerPosition)
        );

        // Create Plans
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoIn
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                hArmUp,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(linearSlides,
                liftUp,
                toClipperLift
        );
        VArmPlan vArmPlan = new VArmPlan(verticalDeposit,
                vArmDown,
                upVArm,
                toClipperVArm
        );
        VClawPlan vClawPlan = new VClawPlan(verticalDeposit,
                releaseVClaw,
                grabVClaw
        );

        // Synchronizer
        return new Synchronizer(
                extendo_plan,
                h_arm_plan,
                h_claw_plan,
                liftPlan,
                vArmPlan,
                vClawPlan
        );
    }

    /**
     * also happens to do klipper job
     * @param targetClipCount
     * @param clipbot
     * @return
     */
    public static Synchronizer getFeederSync(
            int targetClipCount,
            ClipbotSubsystem clipbot
    ) {
        if (targetClipCount <= 0) return new Synchronizer();

        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                (maxClips - targetClipCount) * MFeederConstants.INCHES_PER_CLIP
        );

        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - targetClipCount) * MFeederConstants.INCHES_PER_CLIP
        );


        // Movements
        LinearMFeeder advanceFeeder = new LinearMFeeder(0,
                currentFeederPosition,
                targetFeederPosition
        );

        MoveKlipper klipperClose = new MoveKlipper(
                advanceFeeder.getEndTime() + 1,
                KlipperConstants.closedPosition
        );
        MoveKlipper klipperOpen = new MoveKlipper(
                klipperClose.getEndTime(),
                KlipperConstants.openPosition
        );


        // Plans
        MFeederPlan mFeederPlan = new MFeederPlan(clipbot,
                advanceFeeder
        );

        KlipperPlan klipperPlan = new KlipperPlan(
                clipbot,
                klipperClose, klipperOpen
        );


        // Synchronizer
        return new Synchronizer(
                mFeederPlan,
                klipperPlan
        );
    }

    public static Synchronizer getKlipperSync(
            ClipbotSubsystem clipbot
    ) {
        MoveKlipper klipperClose = new MoveKlipper(0, KlipperConstants.closedPosition);
        MoveKlipper klipperOpen = new MoveKlipper(klipperClose.getEndTime(), KlipperConstants.openPosition);

        KlipperPlan klipperPlan = new KlipperPlan(
                clipbot,
                klipperClose, klipperOpen
        );

        return new Synchronizer(klipperPlan);
    }

    private static TranslationState getCurrentTranslation(LocalizationSubsystem localization) {
        return new TranslationState(localization.getX(), localization.getY());
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians >= Math.PI) radians -= 2*Math.PI;
        while (radians < -Math.PI) radians += 2*Math.PI;
        return radians;
    }
}
