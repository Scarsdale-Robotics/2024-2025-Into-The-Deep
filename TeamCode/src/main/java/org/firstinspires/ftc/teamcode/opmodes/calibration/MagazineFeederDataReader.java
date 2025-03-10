package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;

@Config
@TeleOp(name="Magazine Feeder DATA READER", group="Calibration")
public class MagazineFeederDataReader extends LinearOpMode {

    private Motor magazineFeeder;

    public static double feederPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        boolean magazinePositionReadRequired = AutoToTeleopData.magazineReadRequired;
        if (AutoToTeleopData.magazineReadRequired) {
            MFeederConstants.ZERO_HOME = -AutoToTeleopData.magazinePositionInches;
            AutoToTeleopData.magazineReadRequired = false;
        } else {
            MFeederConstants.ZERO_HOME = MFeederConstants.INCHES_OFFSET;
        }

        while (opModeInInit()) {
            telemetry.addData("AutoToTeleopData.magazinePositionInches", AutoToTeleopData.magazinePositionInches);
            telemetry.addData("magazinePositionReadRequired", magazinePositionReadRequired);
            telemetry.addData("MFeederConstants.ticksToInches(magazineFeeder.getCurrentPosition())", MFeederConstants.ticksToInches(magazineFeeder.getCurrentPosition()));
            telemetry.addData("MFeederConstants.ZERO_HOME", MFeederConstants.ZERO_HOME);
            telemetry.addData("position", MFeederConstants.ticksToInches(magazineFeeder.getCurrentPosition()) - MFeederConstants.ZERO_HOME);
            telemetry.update();
        }

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

            double position = MFeederConstants.ticksToInches(magazineFeeder.getCurrentPosition()) - MFeederConstants.ZERO_HOME;

            telemetry.addData("position", position);
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
    }
}
