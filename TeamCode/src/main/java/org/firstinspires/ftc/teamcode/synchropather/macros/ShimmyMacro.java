package org.firstinspires.ftc.teamcode.synchropather.macros;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

public class ShimmyMacro extends Synchronizer {

    public ShimmyMacro(DriveSubsystem drive, LocalizationSubsystem localization, boolean leftFirst) {
        LinearTranslation shimmyRight, shimmyLeft;
        int magnitude = 2;
        if (leftFirst) {
            shimmyLeft = new LinearTranslation(0,
                    new TranslationState(localization.getPose()),
                    new TranslationState(-magnitude, -24-9+2)
            );
            shimmyRight = new LinearTranslation(shimmyLeft.getEndTime(),
                    new TranslationState(-magnitude, -24-9+2),
                    new TranslationState(magnitude, -24-9+2)
            );
        } else {
            shimmyRight = new LinearTranslation(0,
                    new TranslationState(localization.getPose()),
                    new TranslationState(magnitude, -24-9+2)
            );
            shimmyLeft = new LinearTranslation(shimmyRight.getEndTime(),
                    new TranslationState(magnitude, -24-9+2),
                    new TranslationState(-magnitude, -24-9+2)
            );
        }

        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                shimmyRight,
                shimmyLeft
        );


        LinearRotation rotationStill = new LinearRotation(new TimeSpan(0,1),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(90))
        );

        RotationPlan rotationPlan = new RotationPlan(drive, localization, rotationStill);

        setPlans(translationPlan, rotationPlan);
    }

}
