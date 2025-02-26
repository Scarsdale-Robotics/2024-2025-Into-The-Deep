package org.firstinspires.ftc.teamcode.synchropather.systems.extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

@Config
public class ExtendoPlan extends Plan<ExtendoState> {

    // Feedforward constants
    //TODO: TUNE
    public static double kS = 0.0*ExtendoConstants.MAX_MOTOR_VELOCITY;
    public static double kV = 1;
    public static double kA = 0.05;

    // Positional SQUID constants
    //TODO: TUNE
    public static double kSQU = 4;
    public static double kI = 0;
    public static double kD = 0;

    private double intedt = 0;

    private final ArrayList<Double> eHistory;
    private final ArrayList<Double> dtHistory;
    private final LinearSlidesSubsystem linearSlides;

    private ElapsedTime runtime;
    private final Telemetry telemetry;

    public ExtendoPlan(LinearSlidesSubsystem linearSlides, Movement... movements) {
        super(MovementType.EXTENDO, movements);
        this.linearSlides = linearSlides;
        this.eHistory = new ArrayList<>();
        this.dtHistory = new ArrayList<>();
        this.telemetry = linearSlides.telemetry;
//        robot.telemetry.addData("[SYNCHROPATHER] ExtendoPlan leftHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] ExtendoPlan rightHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] ExtendoPlan desiredState.getHeight()", 0);
//        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        ExtendoState desiredState = getCurrentState();
        ExtendoState desiredVelocity = getCurrentVelocity();
        ExtendoState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getLength();
        double da = desiredAcceleration.getLength();

        // Current state
        double extendoLength = linearSlides.getExtendoPosition();

        // State error
        double e = desiredState.getLength() - extendoLength;
        eHistory.add(e);
        if (eHistory.size() > 5) eHistory.remove(0);

        // Get delta time
        double deltaTime;
        boolean runtimeWasNull = false;
        if (runtime==null) {
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
            intedt += deltaTime * e;
        }
        if (eHistory.size() > 1) {
            if (eHistory.get(eHistory.size() - 2) * e <= 0) {
                // flush integral stack
                intedt = 0;
            }
        }
        if (kI != 0) {
            // Limit integrator to prevent windup
            double integralPowerThreshold = 0.25;
            double integralThresholdBound = Math.abs(integralPowerThreshold * ExtendoConstants.MAX_MOTOR_VELOCITY / kI);
            intedt = bound(intedt, -integralThresholdBound, integralThresholdBound);
        }

        // Error derivatives
        double dedt = 0;
        if (dtHistory.size()==5) {
            if (eHistory.size() == 5) {
                dedt = stencil(eHistory);
            }
        }

        // Control output
        double u = 0;

        // Extendo SQUID
        double squ = Math.signum(e)*Math.sqrt(Math.abs(e));
        u += (kSQU*squ + kI*intedt + kD*dedt) / ExtendoConstants.MAX_MOTOR_VELOCITY;

        // Feedforward
        double fu = (kS*Math.signum(dv) + kV*dv + kA*da) / ExtendoConstants.MAX_MOTOR_VELOCITY;
        u += fu;

        // Set drive powers
        linearSlides.setExtendoPower(u);

        telemetry.addData("[SYNCHROPATHER] ExtendoPlan extendoLength", extendoLength);
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan error", e);
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan desiredState.getLength()", desiredState.getLength());
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan dedt", dedt);
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan intedt", intedt);
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan desiredVelocity", desiredVelocity);
        telemetry.addData("[SYNCHROPATHER] ExtendoPlan desiredAcceleration", desiredAcceleration);
        telemetry.update();

    }

    @Override
    public void stop() {
        linearSlides.stopExtendo();
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