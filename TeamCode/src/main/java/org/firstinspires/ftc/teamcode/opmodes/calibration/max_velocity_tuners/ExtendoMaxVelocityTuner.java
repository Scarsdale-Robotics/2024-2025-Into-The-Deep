package org.firstinspires.ftc.teamcode.opmodes.calibration.max_velocity_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;

import java.util.ArrayList;

@Config
@TeleOp(name="Extendo Max Velocity Tuner", group="Calibration")
public class ExtendoMaxVelocityTuner extends LinearOpMode {

    private Motor extendo;
    public static double SPEED = 1;

    private ArrayList<Double> posHistory;
    private ArrayList<Double> dtHistory;
    private ElapsedTime runtime;

    public double maxExtendoSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        waitForStart();

        posHistory = new ArrayList<>();
        dtHistory = new ArrayList<>();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            power *= SPEED;

            extendo.motor.setPower(power);

            // get pos
            double extendoLength = getExtendoPosition();
            posHistory.add(extendoLength);
            if (posHistory.size() > 5) posHistory.remove(0);

            // Get delta time
            double deltaTime;
            if (runtime==null || runtime.seconds()>0.1) {
                runtime = new ElapsedTime(0);
            } else {
                deltaTime = runtime.seconds();
                runtime.reset();
                dtHistory.add(deltaTime);
                if (dtHistory.size()>5) dtHistory.remove(0);
            }

            // derivative approx
            double extendoVelocity = 0;
            if (dtHistory.size()==5) {
                if (posHistory.size() == 5) {
                    extendoVelocity = stencil(posHistory);
                }
            }

            double extendoSpeed = Math.abs(extendoVelocity);
            maxExtendoSpeed = Math.max(maxExtendoSpeed, extendoSpeed);

            telemetry.addData("power", power);
            telemetry.addData("extendoLength", extendoLength);
            telemetry.addData("extendoVelocity", extendoVelocity);
            telemetry.addData("extendoSpeed", extendoSpeed);
            telemetry.addData("maxExtendoSpeed", maxExtendoSpeed);
            telemetry.update();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620);
        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(true);
    }


    /**
     * @return extendo length in inches, where fully retracted = 0
     */
    public double getExtendoPosition() {
        return extendo.getCurrentPosition() / ExtendoConstants.TICKS_PER_INCH;
    }

    /**
     * @param a The process value array.
     * @return Approximated derivative according to the Five-Point stencil.
     */
    public double stencil(ArrayList<Double> a) {
        double averageDeltaTime = dtHistory.stream().mapToDouble(aa -> aa).average().orElse(0);
        return (-a.get(4) + 8*a.get(3) - 8*a.get(1) + a.get(0)) /
                (12 * averageDeltaTime);
    }
}
