package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;

@Photon
@Config
@TeleOp(name="Magazine Homing Tester", group="Calibration")
public class MagazineHomingTester extends LinearOpMode {

    private Motor magazineFeeder;
    private ClipbotSubsystem clipbot;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        telemetry.addData("[CLIPBOT] homing status", "RUNNING");
        telemetry.update();

        clipbot.homeMagazineFeeder(this, telemetry);

        telemetry.addData("[CLIPBOT] homing status", "DONE");
        telemetry.update();

        while (opModeIsActive()) {
            double totalPower = gamepad1.right_trigger - gamepad1.left_trigger;
            clipbot.setMagazineFeederPower(totalPower);
            telemetry.addData("power", totalPower);

            telemetry.addData("[CLIPBOT] homed position", MFeederConstants.ZERO_HOME);
            telemetry.addData("feeder position (inches, homed)", clipbot.getMagazineFeederPosition());
            telemetry.addData("feeder position (ticks, raw)", magazineFeeder.getCurrentPosition());
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

        clipbot = new ClipbotSubsystem(
                null,
                null,
                magazineFeeder
        );
    }
}
