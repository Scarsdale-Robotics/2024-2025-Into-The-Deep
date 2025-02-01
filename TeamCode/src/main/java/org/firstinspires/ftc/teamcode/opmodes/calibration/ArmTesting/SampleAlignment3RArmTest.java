package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Sample Alignment 3R Arm Test", group="Calibration")
public class SampleAlignment3RArmTest extends LinearOpMode {

    // ARM //
    private Servo servoArmBase;
    private Servo servoArmFirstJoint;
    private Servo servoArmSecondJoint;

    private double targetX = 5; // +X is forward
    private double targetY = 0; // +Y is left
    private double targetZ = 0; // +Z is upward

    // VISION //
    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    private WebcamName cameraName;
    private SampleOrientationProcessor processor;

    public static double deltaHeight = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Init arm
        servoArmBase = hardwareMap.get(ServoImplEx.class, "armBase");
        servoArmFirstJoint = hardwareMap.get(ServoImplEx.class, "armFirstJoint");
        servoArmSecondJoint = hardwareMap.get(ServoImplEx.class, "armSecondJoint");
        Servo3RArmIK servo3RArmIK = new Servo3RArmIK();
        Servo3RArmController servo3RArmController = new Servo3RArmController(
                servoArmBase,
                servoArmFirstJoint,
                servoArmSecondJoint
        );

        // Init vision
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal visionPortal = buildVisionPortal(cameraName);
        waitForStart();
        while ((opModeInInit() || opModeIsActive()) && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);
        sleep(100);
        updateExposure(visionPortal, getCorrectedExposure(processor.getAverageBrightness()));

        while (opModeIsActive()) {
            // Get sample position
            ArrayList<double[]> realPositions = processor.getRealPositions();
            if (processor.getSampleDetected() && !realPositions.isEmpty()) {
                double[] realPosition = realPositions.get(0);
                targetX = -realPosition[1]+10;
                targetY = realPosition[0];
                targetZ = 17-realPosition[2]+deltaHeight;
            } else {
                targetX = 5;
                targetY = 0;
                targetZ = 0;
            }

            // Control arm to target pos
            servo3RArmController.setPosition(servo3RArmIK.getServoArmPositions(
                    new double[]{targetX, targetY, targetZ}
            ));
        }

    }

    public VisionPortal buildVisionPortal(WebcamName cameraName) {
        processor = new SampleOrientationProcessor(telemetry);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessors(processor)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(processor, true);  // let processors run asynchronously using camera data

        return visionPortal;
    }

    private void updateExposure(VisionPortal visionPortal, long ms) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(ms, TimeUnit.MILLISECONDS);  // exposure may have to be adjusted during competitions
    }

    private long getCorrectedExposure(double averageBrightness) {
        if (averageBrightness < 50) return 50;
        else if (averageBrightness < 80) return 27;
        else if (averageBrightness < 100) return 15;
        else if (averageBrightness < 140) return 14;
        else return 5;
    }
}
