package org.firstinspires.ftc.teamcode.synchropather.systems.klipper;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of klipper Movements.
 */
public class KlipperPlan extends Plan<KlipperState> {

    private final ClipbotSubsystem clipbot;

    private final Telemetry telemetry;


    public KlipperPlan(ClipbotSubsystem clipbot, Telemetry telemetry, Movement... movements) {
        super(MovementType.MAGAZINE_INTAKE, movements);
        this.clipbot = clipbot;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MIntakePlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public KlipperPlan(ClipbotSubsystem clipbot, Movement... movements) {
        this(clipbot, null, movements);
    }

    public void loop() {
        // Desired states
        KlipperState desiredState = getCurrentState();

        clipbot.setKlipperPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MIntakePlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
