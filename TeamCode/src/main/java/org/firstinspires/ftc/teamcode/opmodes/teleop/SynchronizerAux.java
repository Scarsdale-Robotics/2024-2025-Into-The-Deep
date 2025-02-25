package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

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


        // Plans
        MFeederPlan mFeederPlan = new MFeederPlan(clipbot,
                advanceFeeder
        );


        // Synchronizer
        return new Synchronizer(
                mFeederPlan
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
}
