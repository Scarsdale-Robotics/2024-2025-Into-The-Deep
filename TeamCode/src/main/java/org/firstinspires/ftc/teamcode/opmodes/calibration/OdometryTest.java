
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

@TeleOp(name="Odometry Test")
public class OdometryTest extends LinearOpMode {
    private DriveSubsystem drive;
    private OdometrySubsystem odometry;
    private final double drive_speed = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
        odometry = new OdometrySubsystem(hardwareMap, this);
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                this,
                telemetry
        );

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPose();
            Pose2d velocity = odometry.getVelocity();

            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("H", pose.getHeading() * 180 / Math.PI);

            telemetry.addData("XV", velocity.getX());
            telemetry.addData("YV", velocity.getY());
            telemetry.addData("HV", velocity.getHeading() * 180 / Math.PI);

            telemetry.addData("LO", odometry.getLeftOdometer().getPosition());
            telemetry.addData("RO", odometry.getRightOdometer().getPosition());
            telemetry.addData("CO", odometry.getCenterOdometer().getPosition());
            telemetry.update();

            ////////////////////
            // DRIVE CONTROLS //
            ////////////////////
            double x = 0, y = 0, turn = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.05)
                x = gamepad1.left_stick_x;
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                y = -gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x) > 0.05)
                turn = gamepad1.right_stick_x;

            /////////////////////////
            // CORRECTION TO (0,0) //
            /////////////////////////

            Pose2d target = new Pose2d(
                        8 * Math.cos(runtime.seconds() / 0.6),
                    8 * Math.sin(runtime.seconds() / 1.2),
                    new Rotation2d(0)
            );


            double kP = 0.5;
            double[] error = {
                    target.getX() - pose.getX(),
                    target.getY() - pose.getY(),
                    normalizeAngle(target.getHeading() - pose.getHeading())
            };

            x += kP * error[0];
            y += kP * error[1];
            turn += 8 * kP * error[2];

            drive.driveFieldCentric(drive_speed * x, drive_speed * y, drive_speed * turn);


        }
        drive.stopController();

    }

    /**
     * Normalizes a given angle to [-pi,pi) degrees.
     * @param degrees the given angle in degrees.
     * @return the normalized angle in degrees.
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (opModeIsActive() && angle <= -Math.PI)
            angle += 2*Math.PI;
        while (opModeIsActive() && angle > Math.PI)
            angle -= 2*Math.PI;
        return angle;
    }

}
