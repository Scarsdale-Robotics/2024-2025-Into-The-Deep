package org.firstinspires.ftc.teamcode.opmodes.calibration.limelight_testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cvprocessors.LimelightDetectorProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightState;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Limelight Raw Data Logger", group="Calibration")
public class LimelightRawLogger extends LinearOpMode {

    private LocalizationSubsystem localization;
    private LinearSlidesSubsystem linearSlides;
    private LimelightSubsystem limelightSubsystem;

    public SampleDataBufferFilter limelightSampleData;

    public static double timeBuffer = 0.045;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,10), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(limelightSubsystem, enableLimelight);
        Synchronizer limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();
        while (opModeIsActive()) {
            localization.update();
            limelightSampleData.updateBufferData();
            limelightAction.update();

            telemetry.addData("localization.getX()", localization.getX());
            telemetry.addData("limelightSampleData.getLastBufferedBotPose().getX()", limelightSampleData.getLastBufferedBotPose().getX());

            telemetry.addData("limelightAction.getElapsedTime()", limelightAction.getElapsedTime());
            telemetry.addData("(LimelightState)limelightAction.getState(MovementType.LIMELIGHT)", (LimelightState)limelightAction.getState(MovementType.LIMELIGHT));
            telemetry.addData("limelightSubsystem.getCurrentState()", limelightSubsystem.getCurrentState());

            ArrayList<Pose2d> detectionPoses = new ArrayList<>();
            List<LLResultTypes.DetectorResult> detections = limelightSubsystem.limelight.getLatestResult().getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                double[] relativePosition = limelightSubsystem.calculateSampleRelativePosition(detection, LimelightConstants.FOV, LimelightConstants.cz, LimelightConstants.theta_incline);
                if (relativePosition != null) {
                    double[] fieldPosition = limelightSubsystem.calculateFieldPosition(limelightSampleData.getLastBufferedBotPose(), relativePosition);
                    detectionPoses.add(new Pose2d(fieldPosition[0], fieldPosition[1], new Rotation2d(fieldPosition[2])));
                }
            }
            telemetry.addData("detections.size()", detections.size());
            telemetry.addData("detectionPoses", detectionPoses);


            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), localization.getPose());
            for (Pose2d samplePose : detectionPoses) {
                Drawing.drawSample(packet.fieldOverlay(), samplePose, "#FFFF00");
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void initSubsystems() {
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
                new Pose2d(),
                pinpoint,
                this,
                telemetry
        );

        // init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelightSubsystem = new LimelightSubsystem(limelight, null);



        this.limelightSampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                timeBuffer,
                1,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
    }
}
