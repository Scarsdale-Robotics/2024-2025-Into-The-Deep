import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class OdometryTest extends LinearOpMode {

    private OdometrySubsystem odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new OdometrySubsystem(hardwareMap, this);

        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d currentPose = odometry.getPose();

            telemetry.addData("Current Pose", currentPose.toString());
            telemetry.update();
        }
    }

    //    @Override
    //    public void loop() {
    //        odometry.update();
    //        Pose2d currentPose = odometry.getPose();
    //        // Use currentPose for navigation or telemetry
    //    }
}