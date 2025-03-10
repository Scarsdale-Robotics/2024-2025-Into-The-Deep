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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cvprocessors.LimelightDetectorProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Config
@TeleOp(name="Only Limelight Tester (no pinpoint)", group="Calibration")
public class LimelightSoloTester extends LinearOpMode {

    private LimelightSubsystem limelightSubsystem;

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    public VisionPortal visionPortal;
    public LimelightDetectorProcessor limelightDetectorProcessor;


    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,1), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(limelightSubsystem, enableLimelight);
        Synchronizer limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();
        while (opModeIsActive()) {
            limelightAction.update();
            limelightSubsystem.update(new Pose2d());
            List<double[]> samplePositions = limelightSubsystem.getYellowSamplePositions();
            if (!samplePositions.isEmpty()) {
                telemetry.addData("SAMPLE X", samplePositions.get(0)[0]);
                telemetry.addData("SAMPLE Y", samplePositions.get(0)[1]);
                telemetry.update();
            }
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init VisionPortal
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.limelightDetectorProcessor = new LimelightDetectorProcessor(this);
        this.visionPortal = buildVisionPortal(cameraName);

        // init limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelightSubsystem = new LimelightSubsystem(limelight, limelightDetectorProcessor);
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
