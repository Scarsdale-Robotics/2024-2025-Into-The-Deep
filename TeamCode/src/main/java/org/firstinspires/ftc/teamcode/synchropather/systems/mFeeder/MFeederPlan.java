package org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

@Config
public class MFeederPlan extends Plan<MFeederState> {

    // Feedforward constants
    //TODO: TUNE
    public static double kS = 0;
    public static double kV = 1;
    public static double kA = 0;

    // Positional SQUID constants
    //TODO: TUNE
    public static double kSQU = 8;
    public static double kI = 0;
    public static double kD = 0;

    private double intedt = 0;

    private final ArrayList<Double> eHistory;
    private final ArrayList<Double> dtHistory;
    private final ClipbotSubsystem clipbot;

    private ElapsedTime runtime;
//    private final Telemetry telemetry;

    public MFeederPlan(ClipbotSubsystem clipbot, Movement... movements) {
        super(MovementType.MAGAZINE_FEEDER, movements);
        this.clipbot = clipbot;
        this.eHistory = new ArrayList<>();
        this.dtHistory = new ArrayList<>();
//        this.telemetry = clipbot.telemetry;
//        robot.telemetry.addData("[SYNCHROPATHER] MFeederPlan leftHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] MFeederPlan rightHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] MFeederPlan desiredState.getHeight()", 0);
//        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        MFeederState desiredState = getCurrentState();
        MFeederState desiredVelocity = getCurrentVelocity();
        MFeederState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getPosition();
        double da = desiredAcceleration.getPosition();

        // Current state
        double feederPosition = clipbot.getMagazineFeederPosition();

        // State error
        double e = desiredState.getPosition() - feederPosition;
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
            double integralThresholdBound = Math.abs(integralPowerThreshold * MFeederConstants.MAX_VELOCITY / kI);
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

        // Magazine feeder SQUID
        double squ = Math.signum(e)*Math.sqrt(Math.abs(e));
        u += (kSQU*squ + kI*intedt + kD*dedt) / MFeederConstants.MAX_VELOCITY;

        // Feedforward
        double fu = (kS*Math.signum(dv) + kV*dv + kA*da) / MFeederConstants.MAX_VELOCITY;
        u += fu;

        // Set motor power
        clipbot.setMagazineFeederPower(u);

//        telemetry.addData("[SYNCHROPATHER] MFeederPlan feederPosition", feederPosition);
//        telemetry.addData("[SYNCHROPATHER] MFeederPlan error", e);
//        telemetry.addData("[SYNCHROPATHER] MFeederPlan desiredState.getLength()", desiredState.getPosition());
//        telemetry.addData("[SYNCHROPATHER] MFeederPlan dedt", dedt);
//        telemetry.addData("[SYNCHROPATHER] MFeederPlan intedt", intedt);
//        telemetry.update();

    }

    @Override
    public void stop() {
        clipbot.stopMagazineFeeder();
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