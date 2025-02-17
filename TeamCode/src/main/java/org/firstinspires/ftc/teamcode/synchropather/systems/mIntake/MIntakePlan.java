package org.firstinspires.ftc.teamcode.synchropather.systems.mIntake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of magazine intake Movements.
 */
public class MIntakePlan extends Plan<MIntakeState> {

    private final ClipbotSubsystem clipbot;

    private final Telemetry telemetry;


    public MIntakePlan(ClipbotSubsystem clipbot, Telemetry telemetry, Movement... movements) {
        super(MovementType.MAGAZINE_INTAKE, movements);
        this.clipbot = clipbot;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MIntakePlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public MIntakePlan(ClipbotSubsystem clipbot, Movement... movements) {
        this(clipbot, null, movements);
    }

    public void loop() {
        // Desired states
        MIntakeState desiredState = getCurrentState();

        clipbot.setMagazineIntakePosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] MIntakePlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
