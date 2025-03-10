package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;

import java.util.ArrayDeque;

@Config
@TeleOp(name="TPS Benchmark", group = "Calibration")
public class TPSBenchmark extends LinearOpMode {

    private AutonomousRobot robot;
    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            updateTPS();
        }

    }


    private void initialize() {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(1,1,new Rotation2d(Math.PI/2)),
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );
        this.telemetry = robot.telemetry;
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.update();
    }
}
