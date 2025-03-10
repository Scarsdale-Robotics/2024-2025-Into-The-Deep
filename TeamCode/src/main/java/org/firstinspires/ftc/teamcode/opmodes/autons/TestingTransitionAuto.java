package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;

@Autonomous(name="TESTING TRANSITION AUTO")
@Disabled
public class TestingTransitionAuto extends LinearOpMode {

    private AutonomousRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();


        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                robot.clipbot.getMagazineFeederPosition()
        );
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - 5) * MFeederConstants.INCHES_PER_CLIP
        );
        LinearMFeeder linearMFeeder = new LinearMFeeder(0, currentFeederPosition, targetFeederPosition);
        MFeederPlan mFeederPlan = new MFeederPlan(robot.clipbot, linearMFeeder);
        Synchronizer advanceFeederAction = new Synchronizer(mFeederPlan);

        advanceFeederAction.start();
        while (opModeIsActive() && advanceFeederAction.update()) {
            updateRobot();
        }
        advanceFeederAction.stop();



    }

    private void initSubsystems() {
        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(24, -72+9, new Rotation2d(Math.toRadians(90))),
                // back against wall, facing towards sub, centered on first seam from middle
                AutonomousRobot.TeamColor.BLUE,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );

        // disable sample orientation processor
        robot.visionPortal.setProcessorEnabled(robot.sampleOrientationProcessor, false);
    }

    private void updateRobot() {
        robot.update();
        robot.overheadSampleData.updateFilterData(robot.overheadCamera.getSamplePositions(), robot.overheadCamera.getSampleAngles(), robot.overheadCamera.getClosestSample()); // Can return null

        // write mag position to static class
        double position = robot.clipbot.getMagazineFeederPosition();
        AutoToTeleopData.magazinePositionInches = position;
        AutoToTeleopData.magazineClipInventory = 5;
        AutoToTeleopData.magazineReadRequired = true;
    }
}
