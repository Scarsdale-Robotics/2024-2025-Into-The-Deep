package org.firstinspires.ftc.teamcode.synchropather.systems.mLoader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of magazine loader Movements.
 */
public class MLoaderPlan extends Plan<MLoaderState> {

    private final ClipbotSubsystem clipbot;

    private final Telemetry telemetry;


    public MLoaderPlan(ClipbotSubsystem clipbot, Telemetry telemetry, Movement... movements) {
        super(MovementType.MAGAZINE_LOADER, movements);
        this.clipbot = clipbot;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MLoaderPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public MLoaderPlan(ClipbotSubsystem clipbot, Movement... movements) {
        this(clipbot, null, movements);
    }

    public void loop() {
        // Desired states
        MLoaderState desiredState = getCurrentState();

        clipbot.setMagazineLoaderPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MLoaderPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
