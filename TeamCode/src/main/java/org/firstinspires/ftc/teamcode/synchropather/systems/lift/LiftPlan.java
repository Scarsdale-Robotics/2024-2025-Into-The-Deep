package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

@Config
public class LiftPlan extends Plan<LiftState> {
    // Feedforward constants
    //TODO: TUNE
    public static double kS = 0;
    public static double kV = 1;
    public static double kA = 0.05;

    // Positional SQUID constants
    //TODO: TUNE
    public static double kSQU = 16;
    public static double kI = 0.0;
    public static double kD = 0.2;

    private double lintedt = 0;
    private double rintedt = 0;

    private final ArrayList<Double> leHistory;
    private final ArrayList<Double> reHistory;
    private final ArrayList<Double> dtHistory;
    private final LinearSlidesSubsystem linearSlides;

    private ElapsedTime runtime;
    private final Telemetry telemetry;

    public LiftPlan(LinearSlidesSubsystem linearSlides, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.linearSlides = linearSlides;
        this.leHistory = new ArrayList<>();
        this.reHistory = new ArrayList<>();
        this.dtHistory = new ArrayList<>();
        this.telemetry = linearSlides.telemetry;
//        robot.telemetry.addData("[SYNCHROPATHER] LiftPlan leftHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] LiftPlan rightHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] LiftPlan desiredState.getHeight()", 0);
//        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        LiftState desiredState = getCurrentState();
        LiftState desiredVelocity = getCurrentVelocity();
        LiftState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getHeight();
        double da = desiredAcceleration.getHeight();

        // Current state
        double leftHeight = linearSlides.getLeftLiftPosition();
        double rightHeight = linearSlides.getRightLiftPosition();
        // lift subsystem get height here instead
        LiftState currentLeftState = new LiftState(leftHeight); // motor position --> height idk
        LiftState currentRightState = new LiftState(rightHeight); // motor position --> height idk

        // State error
        LiftState leftError = desiredState.minus(currentLeftState);
        LiftState rightError = desiredState.minus(currentRightState);
        double le = leftError.getHeight();
        double re = rightError.getHeight();
        leHistory.add(le);
        reHistory.add(re);
        if (leHistory.size() > 5) leHistory.remove(0);
        if (reHistory.size() > 5) reHistory.remove(0);

        // Get delta time
        double deltaTime;
        boolean runtimeWasNull = false;
        if (runtime==null || runtime.seconds()>0.1) {
            runtime = new ElapsedTime(0);
            deltaTime = 0;
            runtimeWasNull = true;
        } else {
            deltaTime = runtime.seconds();
            runtime.reset();
            dtHistory.add(deltaTime);
            if (dtHistory.size()>5) dtHistory.remove(0);
        }

        // Error integrals
        if (!runtimeWasNull) {
            lintedt += deltaTime * le;
            rintedt += deltaTime * re;
        }
        if (leHistory.size() > 1) {
            if (leHistory.get(leHistory.size() - 2) * le <= 0) {
                // flush integral stack
                lintedt = 0;
            }
        }
        if (reHistory.size() > 1) {
            if (reHistory.get(reHistory.size() - 2) * re <= 0) {
                // flush integral stack
                rintedt = 0;
            }
        }
        if (kI != 0) {
            // Limit integrator to prevent windup
            double integralPowerThreshold = 0.25;
            double integralThresholdBound = Math.abs(integralPowerThreshold * LiftConstants.MAX_VELOCITY / kI);
            lintedt = bound(lintedt, -integralThresholdBound, integralThresholdBound);
            rintedt = bound(rintedt, -integralThresholdBound, integralThresholdBound);
        }

        // Error derivatives
        double ldedt = 0;
        double rdedt = 0;
        if (dtHistory.size()==5) {
            if (leHistory.size() == 5) {
                ldedt = stencil(leHistory);
            }
            if (reHistory.size() == 5) {
                rdedt = stencil(reHistory);
            }
        }

        // Control output
        double lu = 0;
        double ru = 0;

        // Lift SQUID
        double lsqu = Math.signum(le)*Math.sqrt(Math.abs(le));
        double rsqu = Math.signum(re)*Math.sqrt(Math.abs(re));
        lu += (kSQU*lsqu + kI*lintedt + kD*ldedt) / LiftConstants.MAX_VELOCITY;
        ru += (kSQU*rsqu + kI*rintedt + kD*rdedt) / LiftConstants.MAX_VELOCITY;

        // Feedforward
        double fu = (kS*Math.signum(dv) + kV*dv + kA*da) / LiftConstants.MAX_VELOCITY;
        lu += fu;
        ru += fu;

        // Set drive powers
        linearSlides.setLeftLiftPower(lu);
        linearSlides.setRightLiftPower(ru);

        telemetry.addData("[SYNCHROPATHER] LiftPlan leftHeight", leftHeight);
        telemetry.addData("[SYNCHROPATHER] LiftPlan rightHeight", rightHeight);
        telemetry.addData("[SYNCHROPATHER] LiftPlan leftError", le);
        telemetry.addData("[SYNCHROPATHER] LiftPlan rightError", re);
        telemetry.addData("[SYNCHROPATHER] LiftPlan desiredState.getHeight()", desiredState.getHeight());
        telemetry.addData("[SYNCHROPATHER] LiftPlan ldedt", ldedt);
        telemetry.addData("[SYNCHROPATHER] LiftPlan rdedt", rdedt);
        telemetry.addData("[SYNCHROPATHER] LiftPlan lintedt", lintedt);
        telemetry.addData("[SYNCHROPATHER] LiftPlan rintedt", rintedt);
        telemetry.update();

    }

    @Override
    public void stop() {
        linearSlides.stopLifts();
    }

    /**
     * @param a The process value array.
     * @return Approximated derivative according to the Five-Point stencil.
     */
    public double stencil(ArrayList<Double> a) {
        double averageDeltaTime = dtHistory.stream().mapToDouble(aa -> aa).average().orElse(0);
        return (-a.get(4) + 8*a.get(3) - 8*a.get(1) + a.get(0)) /
                (12 * averageDeltaTime);
    }

    /**
     * Clips the input x between a given lower and upper bound.
     * @param x
     * @param lower
     * @param upper
     * @return the clipped value of x.
     */
    private static double bound(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }
}