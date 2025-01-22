package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@TeleOp(name="3R Arm Test Opmode", group="Calibration")
public class Servo3RArmTest extends LinearOpMode {

    private Servo servoArmBase;
    private Servo servoArmFirstJoint;
    private Servo servoArmSecondJoint;
    private WebcamName cameraName;

    public static double armBaseZero = 0.53;
    public static double armBaseNinety = 0.85;

    public static double armFirstJointZero = 0.34;
    public static double armFirstJointNinety = 0.66;

    public static double armSecondJointZero = 0.145;
    public static double armSecondJointNinety = 0.45;

    // Measurements in inches
    public static double armFirstSegmentLength = 9;
    public static double armSecondSegmentLength = 9;
    private final double maxRho = armSecondSegmentLength + armFirstSegmentLength;
    private final double minRho = armSecondSegmentLength - armFirstSegmentLength;

    public static double targetX = 5; // +X is forward
    public static double targetY = 0; // +Y is left
    public static double targetZ = 0; // +Z is upward

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        servoArmBase = hardwareMap.get(ServoImplEx.class, "armBase");
        servoArmFirstJoint = hardwareMap.get(ServoImplEx.class, "armFirstJoint");
        servoArmSecondJoint = hardwareMap.get(ServoImplEx.class, "armSecondJoint");
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        waitForStart();

        while (opModeIsActive()) {
            // Convert target cartesian to servo positions
            double[] cartesian = new double[]{targetX, targetY, targetZ};
            double[] spherical = cartesianToSpherical(cartesian);
            double[] servoArmAngles = toServoArmAngles(spherical);
            double[] servoArmPositions = toServoArmPositions(servoArmAngles);

            // Set servo positions
            double alphaPos = servoArmPositions[0];
            double betaPos = servoArmPositions[1];
            double gammaPos = servoArmPositions[2];
            servoArmBase.setPosition(alphaPos);
            servoArmFirstJoint.setPosition(betaPos);
            servoArmSecondJoint.setPosition(gammaPos);

            // Telemetry
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("targetZ", targetZ);
            telemetry.addData("alphaPos", alphaPos);
            telemetry.addData("betaPos", betaPos);
            telemetry.addData("gammaPos", gammaPos);
            telemetry.update();
        }
    }

    private double[] cartesianToSpherical(double[] cartesian) {
        double x = cartesian[0];
        double y = cartesian[1];
        double z = cartesian[2];

        // Convert to spherical
        double rho = Math.sqrt(x*x + y*y + z*z);
        double theta = Math.atan2(y, x);
        double phi = Math.atan2(Math.hypot(x,y), z);
        return new double[]{rho, theta, phi};
    }

    private double[] toServoArmAngles(double[] spherical) {
        double rho = clamp(spherical[0], minRho, maxRho);
        double theta = spherical[1];
        double phi = spherical[2];

        // Convert to arm angles
        double gamma = Math.acos(-(rho*rho)/(2*armFirstSegmentLength*armSecondSegmentLength) + 1);
        double beta = Math.PI - phi - gamma/2;
        double alpha = theta;
        return new double[]{alpha, beta, gamma};
    }

    private double[] toServoArmPositions(double[] servoArmAngles) {
        double alpha = servoArmAngles[0];
        double beta = servoArmAngles[1];
        double gamma = servoArmAngles[2];

        // Convert to arm positions
        double alphaPosition = toServoArmPosition(alpha, armBaseZero, armBaseNinety);
        double betaPosition = toServoArmPosition(beta, armFirstJointZero, armFirstJointNinety);
        double gammaPosition = toServoArmPosition(gamma, armSecondJointZero, armSecondJointNinety);
        return new double[]{alphaPosition, betaPosition, gammaPosition};
    }

    private double toServoArmPosition(double servoArmAngle, double zeroPosition, double ninetyPosition) {
        // Linear interpolation
        double servoArmPosition = zeroPosition + servoArmAngle*(ninetyPosition - zeroPosition)/(Math.PI/2 - 0);
        servoArmPosition = clamp(servoArmPosition, -1, 1);
        return servoArmPosition;
    }

    private double clamp(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }

}
