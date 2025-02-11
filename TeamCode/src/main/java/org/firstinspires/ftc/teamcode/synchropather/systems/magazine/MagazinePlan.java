package org.firstinspires.ftc.teamcode.synchropather.systems.magazine;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

@Config
public class MagazinePlan extends Plan<MagazineState> {

    // Feedforward constants
    //TODO: TUNE
    public static double kS = 6;
    public static double kV = 1;
    public static double kA = 0.2;

    // Positional PD constants
    //TODO: TUNE
    public static double kP = 16;
    public static double kI = 0;
    public static double kD = 1;

    private double intedt = 0;

    private final ArrayList<Double> eHistory;
    private final ArrayList<Double> dtHistory;
    private final ClipbotSubsystem clipbot;

    private ElapsedTime runtime;
    private final Telemetry telemetry;

    public MagazinePlan(ClipbotSubsystem clipbot, Movement... movements) {
        super(MovementType.CLIPBOT_MAGAZINE, movements);
        this.clipbot = clipbot;
        this.eHistory = new ArrayList<>();
        this.dtHistory = new ArrayList<>();
        this.telemetry = clipbot.telemetry;
//        robot.telemetry.addData("[SYNCHROPATHER] MagazinePlan leftHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] MagazinePlan rightHeight", 0);
//        robot.telemetry.addData("[SYNCHROPATHER] MagazinePlan desiredState.getHeight()", 0);
//        robot.telemetry.update();
    }

    public void loop() {
        // Desired states
        MagazineState desiredState = getCurrentState();
        MagazineState desiredVelocity = getCurrentVelocity();
        MagazineState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getLength();
        double da = desiredAcceleration.getLength();

        // Current state
        double magazinePosition = clipbot.getMagazinePosition();

        // State error
        double e = desiredState.getLength() - magazinePosition;
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
            double integralThresholdBound = Math.abs(integralPowerThreshold * MagazineConstants.MAX_VELOCITY / kI);
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

        // Magazine PID
        u += (kP*Math.signum(e)*Math.sqrt(Math.abs(e)) + kI*intedt + kD*dedt) / MagazineConstants.MAX_VELOCITY;

        // Feedforward
        double fu = (kS*Math.signum(dv) + kV*dv + kA*da) / MagazineConstants.MAX_VELOCITY;
        u += fu;

        // Set drive powers
        clipbot.setMagazinePower(u);

        telemetry.addData("[SYNCHROPATHER] MagazinePlan magazinePosition", magazinePosition);
        telemetry.addData("[SYNCHROPATHER] MagazinePlan error", e);
        telemetry.addData("[SYNCHROPATHER] MagazinePlan desiredState.getLength()", desiredState.getLength());
        telemetry.addData("[SYNCHROPATHER] MagazinePlan dedt", dedt);
        telemetry.addData("[SYNCHROPATHER] MagazinePlan intedt", intedt);
        telemetry.update();

    }

    @Override
    public void stop() {
        clipbot.stopMagazine();
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