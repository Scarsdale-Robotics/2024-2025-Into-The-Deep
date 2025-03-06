package org.firstinspires.ftc.teamcode.synchropather.macros;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;

public class ExtendoRetractMacro extends Synchronizer {

    public ExtendoRetractMacro(LinearSlidesSubsystem linearSlides) {
        LinearExtendo stillExtendo = new LinearExtendo(0,
                new ExtendoState(linearSlides.getExtendoPosition()),
                new ExtendoState(LinearSlidesSubsystem.extendoOffset)
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides, stillExtendo);

        setPlans(extendoPlan);
    }

    public ExtendoRetractMacro(LinearSlidesSubsystem linearSlides, HorizontalIntakeSubsystem horizontalIntake) {
        LinearExtendo stillExtendo = new LinearExtendo(0,
                new ExtendoState(linearSlides.getExtendoPosition()),
                new ExtendoState(LinearSlidesSubsystem.extendoOffset)
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides, stillExtendo);

        ReleaseHClaw releaseHClaw = new ReleaseHClaw(0);
        HClawPlan hClawPlan = new HClawPlan(horizontalIntake, releaseHClaw);

        setPlans(extendoPlan, hClawPlan);
    }

}
