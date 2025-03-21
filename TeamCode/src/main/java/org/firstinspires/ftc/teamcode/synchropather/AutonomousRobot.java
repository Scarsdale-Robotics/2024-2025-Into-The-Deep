package org.firstinspires.ftc.teamcode.synchropather;


import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cvprocessors.ClawVacancyProcessor;
import org.firstinspires.ftc.teamcode.cvprocessors.LimelightDetectorProcessor;
import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

// TODO: merge with robotsystem later(?)
// nathan i am sorry
//viir: pls do this, it makes the code much easier to understand.

public class AutonomousRobot {

    public enum TeamColor {
        RED(),
        BLUE()
    }

    public final LinearOpMode opMode;
    public final Telemetry telemetry;
    public final TeamColor teamColor;

    public final DriveSubsystem drive;
    public final LocalizationSubsystem localization;
    public final HorizontalIntakeSubsystem horizontalIntake;
    public final VerticalDepositSubsystem verticalDeposit;
    public final OverheadCameraSubsystem overheadCamera;
    public final LinearSlidesSubsystem linearSlides;
    public final ClipbotSubsystem clipbot;
    public final LimelightSubsystem limelightSubsystem;

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    public final VisionPortal visionPortal;
    public final SampleOrientationProcessor sampleOrientationProcessor;
    public final LimelightDetectorProcessor limelightDetectorProcessor;
    public final ClawVacancyProcessor clawVacancyProcessor;
    public SampleDataBufferFilter.SampleTargetingMethod sampleTargetingMethod;

    public SampleDataBufferFilter overheadSampleData;
    public SampleDataBufferFilter limelightSampleData;


    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // TODO =========== Update HardwareRobot when hardware is finalized ========= TODO
    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

    public AutonomousRobot(HardwareMap hardwareMap, Pose2d initialPose, TeamColor teamColor, LinearOpMode opMode, SampleDataBufferFilter.SampleTargetingMethod targetingMethod) {
        this.opMode = opMode;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.teamColor = teamColor;
//        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap); // TODO refer above


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


        // init vertical deposit
        Servo leftDepositArm = hardwareMap.get(ServoImplEx.class, "leftDepositArm");
        Servo rightDepositArm = hardwareMap.get(ServoImplEx.class, "rightDepositArm");
        Servo depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");
        this.verticalDeposit = new VerticalDepositSubsystem(
                leftDepositArm,
                rightDepositArm,
                depositClaw
        );


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


        // init VisionPortal
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.sampleOrientationProcessor = new SampleOrientationProcessor();
        this.limelightDetectorProcessor = new LimelightDetectorProcessor(this.opMode);
        this.clawVacancyProcessor = new ClawVacancyProcessor();
        this.visionPortal = buildVisionPortal(cameraName);

        // init overhead camera
        this.overheadCamera = new OverheadCameraSubsystem(visionPortal, sampleOrientationProcessor, telemetry);

        // init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelightSubsystem = new LimelightSubsystem(limelight, limelightDetectorProcessor);


        // init localization
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.localization = new LocalizationSubsystem(
                initialPose,
                pinpoint,
                this.opMode,
                telemetry
        );


        // init sample data buffer filters
        this.sampleTargetingMethod = targetingMethod;
        overheadSampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                0.04375,
                3,
                targetingMethod
        );
        limelightSampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                0.15,
                1,
                targetingMethod
        );


        // Clipbot
        Servo magazineIntake = hardwareMap.get(ServoImplEx.class, "magazineIntake");
        Servo magazineLoader1 = hardwareMap.get(ServoImplEx.class, "magazineLoaderClose");
        Servo magazineLoader2 = hardwareMap.get(ServoImplEx.class, "magazineLoaderFar");

        Motor magazineFeeder = new MotorEx(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620);
        magazineFeeder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineFeeder.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazineFeeder.setRunMode(Motor.RunMode.RawPower);
        magazineFeeder.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setInverted(false);

        this.clipbot = new ClipbotSubsystem(
                magazineIntake,
                magazineLoader1,
                magazineLoader2,
                initServo(hardwareMap, "klipper"),
                magazineFeeder,
                telemetry,
                opMode
        );



        // correct camera exposure
        overheadCamera.correctExposure(this.opMode, telemetry);
    }

    private VisionPortal buildVisionPortal(WebcamName cameraName) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(true)
                .addProcessors(sampleOrientationProcessor, limelightDetectorProcessor, clawVacancyProcessor)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(sampleOrientationProcessor, true);
        visionPortal.setProcessorEnabled(limelightDetectorProcessor, false);
        visionPortal.setProcessorEnabled(clawVacancyProcessor, false);

        return visionPortal;
    }

    public void setOverheadSampleDataBufferFilter(SampleDataBufferFilter overheadSampleData) {
        this.overheadSampleData = overheadSampleData;
    }

    public void update() {
        localization.update();
        linearSlides.update();
        overheadSampleData.updateBufferData();
        limelightSampleData.updateBufferData();
        limelightSubsystem.update(limelightSampleData);
        clipbot.update();
    }

}
