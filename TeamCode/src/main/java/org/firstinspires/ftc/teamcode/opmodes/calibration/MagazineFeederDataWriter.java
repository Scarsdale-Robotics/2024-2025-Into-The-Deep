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
@TeleOp(name="Magazine Feeder DATA WRITER", group="Calibration")
public class MagazineFeederDataWriter extends LinearOpMode {

    private Motor magazineFeeder;

    public static double feederPower = 0;

    private double magazinePosition = 0;
    private double previousMagazinePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        MFeederConstants.ZERO_HOME = MFeederConstants.INCHES_OFFSET;
        AutoToTeleopData.magazineReadRequired = true;

        while (opModeIsActive()) {
            update();

            double totalPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (gamepad1.right_trigger!=0 || gamepad1.left_trigger!=0) {
                magazineFeeder.set(totalPower);
                telemetry.addData("power", totalPower);
            } else {
                magazineFeeder.set(feederPower);
                telemetry.addData("power", feederPower);
            }


            double position = MFeederConstants.ticksToInches(magazinePosition) - MFeederConstants.ZERO_HOME;

            AutoToTeleopData.magazinePositionInches = position;

            telemetry.addData("position", position);
            telemetry.update();
        }

    }

    private void update() {
        double obtainedPosition = magazineFeeder.getCurrentPosition();
        if ((obtainedPosition!=0 || previousMagazinePosition==0) && !isStopRequested()) {
            magazinePosition = obtainedPosition;
        }
        previousMagazinePosition = obtainedPosition;
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
