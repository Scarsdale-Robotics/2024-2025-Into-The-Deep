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
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements.MoveKlipper;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

import java.util.ArrayDeque;

@Autonomous(name="Transfer and Clip Motion", group = "Calibration")
public class TransferAndClipMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private AutonomousRobot robot;

    private int clipInventory = MFeederConstants.MAX_CAPACITY;
    private boolean inventoryStocked = true;

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

        robot.verticalDeposit.setArmPosition(0.5);
        robot.verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
        robot.clipbot.setKlipperPosition(KlipperConstants.openPosition);
    }

    private void initAdvanceMagazineMotion() {
        if (!inventoryStocked) return;
        if (clipInventory <= 0) {
            inventoryStocked = false;
            return;
        }

        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );


        // Grab onto sample
        GrabVClaw grabVClaw = new GrabVClaw(0);

        // Keep klipper open
        MoveKlipper initKlipper = new MoveKlipper(0, KlipperConstants.openPosition);

        // Lower v arm to spec maker
        LinearVArm lowerVArm = new LinearVArm(0,
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Advance feeder
        LinearMFeeder advanceFeeder = new LinearMFeeder(lowerVArm.getEndTime(),
                currentFeederPosition,
                targetFeederPosition
        );

        // KLIP
        MoveKlipper klipSample = new MoveKlipper(advanceFeeder.getEndTime(), KlipperConstants.closedPosition);

        // UNKLIP
        MoveKlipper unklipSample = new MoveKlipper(klipSample.getEndTime(), KlipperConstants.openPosition);

        // Take specimen out
        LinearVArm raiseVArm = new LinearVArm(unklipSample.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(0.5)
        );

        // Release specimen
        ReleaseVClaw releaseVClaw = new ReleaseVClaw(raiseVArm.getEndTime());


        //// Plans
        MFeederPlan mFeederPlan;
        if (clipInventory==0) {
            LinearMFeeder resetFeeder = new LinearMFeeder(advanceFeeder.getEndTime(),
                    targetFeederPosition,
                    new MFeederState(0)
            );
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder,
                    resetFeeder
            );
            inventoryStocked = false;
            clipInventory = MFeederConstants.MAX_CAPACITY;
        } else {
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder
            );
        }

        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                lowerVArm,
                raiseVArm
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                grabVClaw,
                releaseVClaw
        );

        KlipperPlan klipperPlan = new KlipperPlan(robot.clipbot,
                initKlipper,
                klipSample,
                unklipSample
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                mFeederPlan,
                vArmPlan,
                vClawPlan,
                klipperPlan
        );
    }

}
