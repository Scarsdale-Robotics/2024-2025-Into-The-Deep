package org.firstinspires.ftc.teamcode.synchropather.systems.hWrist;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of horizontal wrist Movements.
 */
public class HWristPlan extends Plan<HWristState> {

    private final HorizontalIntakeSubsystem horizontalIntake;

    private final Telemetry telemetry;


    public HWristPlan(HorizontalIntakeSubsystem horizontalIntake, Telemetry telemetry, Movement... movements) {
        super(MovementType.HORIZONTAL_WRIST, movements);
        this.horizontalIntake = horizontalIntake;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HWristPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public HWristPlan(HorizontalIntakeSubsystem horizontalIntake, Movement... movements) {
        this(horizontalIntake, null, movements);
    }

    public void loop() {
        // Desired states
        HWristState desiredState = getCurrentState();

        horizontalIntake.setWristAngle(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HWristPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
