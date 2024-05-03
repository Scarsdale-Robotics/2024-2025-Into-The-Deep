package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OdometryTest2 extends LinearOpMode {

    private OdometrySubsystem odometry;
    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new OdometrySubsystem(hardwareMap, this);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d currentPose = odometry.getPose();

            telemetry.addData("Current Pose", currentPose.toString());
            telemetry.update();

            // Sample movement: move straight for 2 seconds, then turn for 2 seconds
            moveStraight(1.0, 2000);
            turn(1.0, 2000);
        }
    }

    private void moveStraight(double power, long time) throws InterruptedException {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        Thread.sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void turn(double power, long time) throws InterruptedException {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
        Thread.sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}