package org.firstinspires.ftc.teamcode.opmodes.calibration.combined_testing;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cvprocessors.ClawVacancyProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Claw Vacancy Logger", group = "Calibration")
public class ClawVacancyLogger extends LinearOpMode {

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    public VisionPortal visionPortal;
    public ClawVacancyProcessor clawVacancyProcessor;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            // Telemetry for claw vacancy processor
            int clawPixelCount = clawVacancyProcessor.getPixelCount();
            boolean clawEmpty = clawVacancyProcessor.isClawEmpty();
            telemetry.addData("clawPixelCount", clawPixelCount);
            telemetry.addData("clawEmpty", clawEmpty);
            telemetry.update();
        }
    }


    private void initialize() {
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.clawVacancyProcessor = new ClawVacancyProcessor();
        this.visionPortal = buildVisionPortal(cameraName);
    }

    private VisionPortal buildVisionPortal(WebcamName cameraName) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(true)
                .addProcessors(clawVacancyProcessor)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(clawVacancyProcessor, true);

        return visionPortal;
    }

}
