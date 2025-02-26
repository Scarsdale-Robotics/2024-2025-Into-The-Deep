package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;

import java.util.ArrayDeque;

@Autonomous(name="Transfer and Clip Motion", group = "Calibration")
public class TransferAndClipMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private AutonomousRobot robot;

    private int clipInventory = MFeederConstants.MAX_CAPACITY;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();


        waitForStart();

        while (opModeIsActive()) {
//            // Wait for button press
//            while (opModeIsActive() && gamepad1.triangle) updateTPS();
//            while (opModeIsActive() && !gamepad1.triangle) updateTPS();
//            if (clipInventory <= 0) continue;
//            initAdvanceMagazineMotion();
//
//            // pick up
//            synchronizer.start();
//            while (opModeIsActive() && synchronizer.update()) {
//                updateTPS();
//            }
//            while (opModeIsActive() && !gamepad1.triangle) {
//                updateTPS();
//                synchronizer.update();
//            }
//            synchronizer.stop();
//            updateTPS();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(),
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );
    }

    private void initAdvanceMagazineMotion() {
        if (clipInventory <= 0) return;

        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );


        // Movements
        LinearMFeeder advanceFeeder = new LinearMFeeder(0,
                currentFeederPosition,
                targetFeederPosition
        );


        // Plans
        MFeederPlan mFeederPlan = new MFeederPlan(robot.clipbot,
                advanceFeeder
        );


        // Synchronizer
        this.synchronizer = new Synchronizer(
                mFeederPlan
        );
    }

}
