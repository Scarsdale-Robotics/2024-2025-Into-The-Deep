package org.firstinspires.ftc.teamcode.opmodes.calibration.max_velocity_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import java.util.ArrayList;

@Config
@TeleOp(name="Drive Max Velocity/Acceleration Tuner", group="Calibration")
public class DriveMaxVelocityTuner extends LinearOpMode {

    public double maxTranslationSpeed = 0;
    public double maxRotationSpeed = 0;

    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    private ArrayList<double[]> translationVeloHistory;
    private ArrayList<Double> rotationVeloHistory;
    private ArrayList<Double> dtHistory;
    private ElapsedTime runtime;

    public double maxTranslationAcceleration = 0;
    public double maxRotationAcceleration = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems(new Pose2d());

        translationVeloHistory = new ArrayList<>();
        rotationVeloHistory = new ArrayList<>();
        dtHistory = new ArrayList<>();

        telemetry.addData("velocity.getX()", 0);
        telemetry.addData("velocity.getY()", 0);
        telemetry.addData("velocity.getHeading()", 0);
        telemetry.addData("translationSpeed", 0);
        telemetry.addData("rotationSpeed", 0);
        telemetry.addData("maxTranslationSpeed", 0);
        telemetry.addData("maxRotationSpeed", 0);
        telemetry.addData("maxTranslationAcceleration", 0);
        telemetry.addData("rotationAbsAcceleration", 0);
        telemetry.addData("maxRotationAcceleration", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controlDrive();
            Pose2d velocity = localization.getVelocity();
            translationVeloHistory.add(new double[]{velocity.getX(), velocity.getY()});
            if (translationVeloHistory.size() > 6) translationVeloHistory.remove(0);
            rotationVeloHistory.add(velocity.getHeading());
            if (rotationVeloHistory.size() > 5) rotationVeloHistory.remove(0);

            double translationSpeed = Math.hypot(velocity.getX(), velocity.getY());
            double rotationSpeed = Math.abs(velocity.getHeading());

            maxTranslationSpeed = Math.max(maxTranslationSpeed, translationSpeed);
            maxRotationSpeed = Math.max(maxRotationSpeed, rotationSpeed);

            // Get delta time
            double deltaTime;
            if (runtime==null || runtime.seconds()>0.1) {
                runtime = new ElapsedTime(0);
            } else {
                deltaTime = runtime.seconds();
                runtime.reset();
                dtHistory.add(deltaTime);
                if (dtHistory.size()>5) dtHistory.remove(0);
            }

            // translation acceleration approx
            double translationAcceleration = 0;
            if (dtHistory.size()==5) {
                if (translationVeloHistory.size() == 6) {
                    // differentiate along arclength
                    ArrayList<Double> norm_dv_hist = new ArrayList<>();
                    double norm_dv_arclength = 0;
                    for (int i = 1; i <= 5; i++) {
                        double[] v0 = translationVeloHistory.get(i-1);
                        double[] v1 = translationVeloHistory.get(i);
                        double dx = v1[0] - v0[0];
                        double dy = v1[1] - v0[1];
                        norm_dv_arclength += Math.hypot(dx,dy);
                        norm_dv_hist.add(norm_dv_arclength);
                    }
                    translationAcceleration = stencil(norm_dv_hist);
                }
            }
            maxTranslationAcceleration = Math.max(maxTranslationAcceleration, translationAcceleration);

            // rotation acceleration approx
            double rotationAcceleration = 0;
            if (dtHistory.size()==5) {
                if (rotationVeloHistory.size() == 5) {
                    rotationAcceleration = stencil(rotationVeloHistory);
                }
            }
            double rotationAbsAcceleration = Math.abs(rotationAcceleration);
            maxRotationAcceleration = Math.max(maxRotationAcceleration, rotationAbsAcceleration);


            // telemetry
            telemetry.addData("velocity.getX()", velocity.getX());
            telemetry.addData("velocity.getY()", velocity.getY());
            telemetry.addData("velocity.getHeading()", velocity.getHeading());
            telemetry.addData("translationSpeed", translationSpeed);
            telemetry.addData("rotationSpeed", rotationSpeed);
            telemetry.addData("maxTranslationSpeed", maxTranslationSpeed);
            telemetry.addData("maxRotationSpeed", maxRotationSpeed);
            telemetry.addData("maxTranslationAcceleration", maxTranslationAcceleration);
            telemetry.addData("rotationAbsAcceleration", rotationAbsAcceleration);
            telemetry.addData("maxRotationAcceleration", maxRotationAcceleration);
            telemetry.update();
        }

    }

    /**
     * Field centric drive (based on driver POV)
     */
    private void controlDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
//        drive.driveFieldCentricPowers(forward, strafe, turn, Math.toDegrees(localization.getH()));
        drive.driveRobotCentricPowers(forward, strafe, turn);
    }

    private void initSubsystems(Pose2d initialPose) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init drive
        MotorEx leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        MotorEx rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        MotorEx leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        MotorEx rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );


        // init localization
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.localization = new LocalizationSubsystem(
                initialPose,
                pinpoint,
                this,
                telemetry
        );


        //// init initial pose
        telemetry.addData("status", "setting pinpoint initialPose");
        telemetry.update();

        // wait for pinpoint to get ready
        while (opModeIsActive() && pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY);

        // create comparators
        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH, initialPose.getX(), initialPose.getY(), AngleUnit.RADIANS, initialPose.getHeading());
        double initialX = initialPose.getX();
        double initialY = initialPose.getY();
        double initialH = initialPose.getHeading();
        Pose2D ppPose = pinpoint.getPosition();
        double ppX = ppPose.getX(DistanceUnit.INCH);
        double ppY = ppPose.getY(DistanceUnit.INCH);
        double ppH = ppPose.getHeading(AngleUnit.RADIANS);

        // wait until initial position has been set
        while (!(equal(ppX,initialX) && equal(ppY,initialY) && equal(ppH,initialH))) {
            pinpoint.setPosition(initialPose2D);
            pinpoint.update();
            ppPose = pinpoint.getPosition();
            ppX = ppPose.getX(DistanceUnit.INCH);
            ppY = ppPose.getY(DistanceUnit.INCH);
            ppH = ppPose.getHeading(AngleUnit.RADIANS);

            // Telemetry
            telemetry.addData("status", "setting pinpoint initialPose");
            telemetry.addData("(status) ppX", ppX);
            telemetry.addData("(status) ppY", ppY);
            telemetry.addData("(status) ppH", ppH);
            telemetry.update();
        }
        telemetry.addData("status", "finished init");
        telemetry.update();

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

    /**
     * Determines whether the two inputs are approximately equal to each other
     * within an epsilon of 1e-3
     * @param a
     * @param b
     * @return Math.abs(a-b) <= 1e-3
     */
    private static boolean equal(double a, double b) {
        return Math.abs(a-b) <= 1e-3;
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

}
