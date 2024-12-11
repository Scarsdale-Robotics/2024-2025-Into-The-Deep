package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2R IK Arm Test", group="Calibration")
@Config
public class IKArmTest extends LinearOpMode {

    // Servo keypoints
    public static double armBetaZeroPosition = 0.74;
    public static double armBetaPIPosition = 0.08;

    public static double armGammaZeroPosition = 0.81;
    public static double armGammaPIPosition = 0.15;

    // Specs in inches
    public static double armLength1 = 10.25;
    public static double armLength2 = 10.5;

    public static double armXPosition = 8.5;
    public static double armZPosition = 0;

    // Servos
    Servo armBeta;
    Servo armGamma;

    public static double servoArmBetaPosition;
    public static double servoArmGammaPosition;


    public static double rho = 0;
    public static double phi = 0;
    public static double beta = 0;
    public static double gamma = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        armBeta = hardwareMap.get(ServoImplEx.class, "armBeta");
        armGamma = hardwareMap.get(ServoImplEx.class, "armGamma");

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        double period = 10; // seconds
        boolean circlePressed = false;
        while (opModeIsActive()) {

            double multiplier = 0.05;
            armXPosition += multiplier*gamepad1.left_stick_x;
            armZPosition += -multiplier*gamepad1.left_stick_y;

            double[] servoAngles;

            if (!circlePressed && gamepad1.circle) runtime.reset();
            if (gamepad1.circle) {
                double theta = 2*Math.PI * runtime.seconds() / period;
                double xt = 6+2.5*Math.cos(2*theta)*Math.cos(theta);
                double zt = 2.5*Math.cos(2*theta)*Math.sin(theta);

                servoAngles = calculateIK(xt, zt);
            } else {
                servoAngles = calculateIK(armXPosition, armZPosition);
            }
            circlePressed = gamepad1.circle;


            servoArmBetaPosition = servoAngles[1];
            servoArmGammaPosition = servoAngles[2];

            telemetry.addData("alpha", servoAngles[0]);
            telemetry.addData("beta", servoArmBetaPosition);
            telemetry.addData("gamma", servoArmGammaPosition);
            telemetry.update();



            setArmBetaPosition(servoArmBetaPosition);
            setArmGammaPosition(servoArmGammaPosition);


        }


    }

    private void setArmBetaPosition(double radians) {
        double armBetaPerRadian = (armBetaPIPosition - armBetaZeroPosition) / Math.PI;
        double servoPosition = armBetaZeroPosition + radians * armBetaPerRadian;
        servoPosition = clamp(servoPosition, 0, 1);
        armBeta.setPosition(servoPosition);
    }

    private void setArmGammaPosition(double radians) {
        double armGammaPerRadian = (armGammaPIPosition - armGammaZeroPosition) / Math.PI;
        double servoPosition = armGammaZeroPosition + radians * armGammaPerRadian;
        servoPosition = clamp(servoPosition, 0, 1);
        armGamma.setPosition(servoPosition);
    }

    /**
     * Converts cartesian coordinates to 2R IK Arm angles (in radians)
     * @param xPosition inches.
     * @param zPosition inches.
     * @return double[]{alpha, beta, gamma}.
     */
    private double[] calculateIK(double xPosition, double zPosition) {
        double rhoMin = Math.sqrt(Math.max(0,armLength1*armLength1 - armLength2*armLength2));
        double rhoMax = armLength1 + armLength2;

        double rho = clamp(Math.hypot(xPosition, zPosition), rhoMin, rhoMax);
        double phi = Math.atan2(Math.abs(xPosition), zPosition);

        double gamma = Math.acos(((rho*rho) / (-2*armLength1*armLength2)) + 1);
        double beta = Math.PI - phi - gamma / 2;

        IKArmTest.rho = rho;
        IKArmTest.phi = phi;
        IKArmTest.beta = beta;
        IKArmTest.gamma = gamma;

        return new double[]{0, beta, gamma};
    }

    private double clamp(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }


}
