package org.firstinspires.ftc.teamcode.opmodes.calibration.limelight_testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.cvprocessors.LimelightDetectorProcessor;
import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Limelight Lag Logger", group="Calibration")
public class LimelightLagLogger extends LinearOpMode {

    private LocalizationSubsystem localization;
    private LinearSlidesSubsystem linearSlides;
    private LimelightSubsystem limelightSubsystem;

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    public VisionPortal visionPortal;
    public LimelightDetectorProcessor limelightDetectorProcessor;

    public SampleDataBufferFilter limelightSampleData;

    public static double timeBuffer = 0.045;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems(new Pose2d(0,0,new Rotation2d(0)));

        waitForStart();

        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,1), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(limelightSubsystem, enableLimelight);
        Synchronizer limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();
        while (opModeIsActive()) {
            localization.update();
            limelightSampleData.updateBufferData();
            limelightSubsystem.update(limelightSampleData);
            limelightAction.update();
            List<double[]> samplePositions = limelightSubsystem.getYellowSamplePositions();
            if (!samplePositions.isEmpty()) {
                telemetry.addData("SAMPLE X", samplePositions.get(0)[0]);
                telemetry.addData("SAMPLE Y", samplePositions.get(0)[1]);
                telemetry.update();
            }
        }
    }

    private void initSubsystems(Pose2d initialPose) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init linear slides
        Motor extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_312);
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

        // init VisionPortal
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.limelightDetectorProcessor = new LimelightDetectorProcessor(this);
        this.visionPortal = buildVisionPortal(cameraName);

        // init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelightSubsystem = new LimelightSubsystem(limelight, limelightDetectorProcessor);

        telemetry.addData("limelight==null", limelight==null);
        telemetry.update();

        this.limelightSampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                timeBuffer,
                1,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
    }

    private VisionPortal buildVisionPortal(WebcamName cameraName) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessors(limelightDetectorProcessor)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(limelightDetectorProcessor, true);

        return visionPortal;
    }
}
