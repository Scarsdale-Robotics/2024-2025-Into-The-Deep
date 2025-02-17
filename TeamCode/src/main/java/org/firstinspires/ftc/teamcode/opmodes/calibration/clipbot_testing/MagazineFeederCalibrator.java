package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Photon
@Config
@TeleOp(name="Magazine Feeder Calibrator", group="Calibration")
public class MagazineFeederCalibrator extends LinearOpMode {

    private Motor magazineFeeder;
    private PhotonDcMotor magazineFeederCurrentSensor;

    public static double feederPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        while (opModeIsActive()) {
            double totalPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (gamepad1.right_trigger!=0 || gamepad1.left_trigger!=0) {
                magazineFeeder.set(totalPower);
                telemetry.addData("power", totalPower);
            } else {
                magazineFeeder.set(feederPower);
                telemetry.addData("power", feederPower);
            }

            double position = magazineFeeder.getCurrentPosition();
            double current = magazineFeederCurrentSensor.getCorrectedCurrent(CurrentUnit.MILLIAMPS);

            telemetry.addData("position", position);
            telemetry.addData("current [mA]", current);
            telemetry.update();
        }

    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        magazineFeeder = new MotorEx(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620);
        magazineFeeder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineFeeder.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazineFeeder.setRunMode(Motor.RunMode.RawPower);
        magazineFeeder.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setInverted(false);

        this.magazineFeederCurrentSensor = (PhotonDcMotor) magazineFeeder.motor;
    }
}
