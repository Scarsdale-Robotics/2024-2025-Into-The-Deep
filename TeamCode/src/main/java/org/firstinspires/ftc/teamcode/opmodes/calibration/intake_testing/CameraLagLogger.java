package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;

import java.util.ArrayDeque;

public class CameraLagLogger extends LinearOpMode {

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    public static double timeBuffer = 0;
    private double[] lastBufferedExtendoPosition;  // {position, timestamp}

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems(new Pose2d(0,0,new Rotation2d(0)));

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        lastBufferedExtendoPosition = new double[] {
                linearSlides.getExtendoPosition(),
                runtime.seconds()
        };

        waitForStart();
        overheadCamera.correctExposure();

        while (opModeIsActive()) {
            updateTPS();
            double[] closestSample = overheadCamera.getClosestSample();
            if (closestSample == null) {
                telemetry.addData("sample pos", "[NONE]");
                telemetry.update();
                continue;
            }
            double extendoPosition = lastBufferedExtendoPosition[0];
            double x_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[0] + extendoPosition + closestSample[0];
            telemetry.addData("sample pos", x_sample_bot);
            telemetry.update();
        }
    }

    private void initSubsystems(Pose2d initialPose) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init horizontal intake
        Servo leftHorizontalArm = hardwareMap.get(ServoImplEx.class, "leftHorizontalArm");
        Servo rightHorizontalArm = hardwareMap.get(ServoImplEx.class, "rightHorizontalArm");
        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");
        Servo horizontalClaw = hardwareMap.get(ServoImplEx.class, "horizontalClaw");
        this.horizontalIntake = new HorizontalIntakeSubsystem(
                leftHorizontalArm,
                rightHorizontalArm,
                horizontalWrist,
                horizontalClaw
        );
        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.9);

        // init overhead camera
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.overheadCamera = new OverheadCameraSubsystem(cameraName, telemetry);

        // init linear slides
        // TODO: FIGURE OUT WHAT RPM EXTENDO MOTOR IS
        Motor extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620);
        Motor leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        Motor rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(true);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(false);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);

        this.linearSlides = new LinearSlidesSubsystem(extendo, leftLift, rightLift, telemetry);

        // init localization
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.localization = new LocalizationSubsystem(
                initialPose,
                pinpoint,
                this,
                telemetry
        );

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

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        localization.update();
        telemetry.update();

        if (currentTime - lastBufferedExtendoPosition[1] > timeBuffer) {
            lastBufferedExtendoPosition = new double[] {
                    linearSlides.getExtendoPosition(),
                    runtime.seconds()
            };
        }
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
}
