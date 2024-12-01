package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Autonomous(name="[TESTING] Auto Blue Observation 1+4", group = "Autons")
public class TESTING_AutoBlueObservation1Plus4 extends LinearOpMode {
    RobotSystem robot;
    Synchronizer synchronizer;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowPartialUp = 0.34;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(-24, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
        robot.inDep.setClawPosition(clawClosed);
        robot.inDep.setElbowPosition(elbowUp-0.2);
        initSynchronizer();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) {
            robot.logOdometry();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
            if (robot.opMode.gamepad1.triangle)
                Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        synchronizer.stop();
    }


    private void initSynchronizer() {

        ///////////////////////////////////////////////////////////////////(STEP 1)
        //// Drive to submersible to deposit preloaded specimen

        CRSplineTranslation spline1 = new CRSplineTranslation(0, //move to submersible
                new TranslationState(-24,63.5),
                new TranslationState(-12, 50),
                new TranslationState(-10, 37)
        );

        LinearRotation still = new LinearRotation(0, // does not rotate
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-90))
        );


        LinearLift liftPreload1 = new LinearLift(spline1.getStartTime(), //lifts the lift to prepare to place specimen
                new LiftState(0),
                new LiftState(1400)
        );

        LinearLift liftPreload2 = new LinearLift(liftPreload1.getEndTime(), //lowers to lift to place specimen
                new LiftState(1400),
                new LiftState(0)
        );

        // claw
        LinearClaw claw1 = new LinearClaw(liftPreload2.getStartTime()+.66, //opens the claw to release specimen
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //elbow
        LinearElbow elbowStill = new LinearElbow(claw1.getStartTime(),
                new ElbowState(elbowUp),
                new ElbowState(elbowPartialUp)
        );


        ///////////////////////////////////////////////////////////////////(STEP 2)
        //// Turn, grab, and then place all 3 samples into observation zone

        //T,G, and P the 1st sample
        CRSplineTranslation splineSampleToObservation1 = new CRSplineTranslation(liftPreload2.getEndTime(), //moves into position to get first sample
                new TranslationState(-10, 37),
                new TranslationState(-37.5, 38.5)
        );

        LinearRotation rotateSampleToObservation1 = new LinearRotation(liftPreload2.getEndTime(), //rotates to first sample
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-129.671))
        );

        LinearElbow elbowSampleToObservation1 = new LinearElbow(rotateSampleToObservation1.getEndTime(), //goes down to sample
                new ElbowState(elbowPartialUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawSampleToObservation1 = new LinearClaw(elbowSampleToObservation1.getEndTime(), //grabs the sample
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        CRSplineTranslation splineSampleToObservation1p5 = new CRSplineTranslation(clawSampleToObservation1.getEndTime() + 0.44, //moves to observation and then to second sample
                new TranslationState(-37.5, 38.5),
                new TranslationState(-42.75, 48),
                new TranslationState(-47.5, 38.5)
        );

        LinearRotation rotateSampleToObservation1p5 = new LinearRotation(clawSampleToObservation1.getEndTime(), //rotates to observation
                new RotationState(Math.toRadians(-129.671)),
                new RotationState(Math.toRadians(-219))
        );

        LinearClaw clawSampleToObservation1p5 = new LinearClaw(rotateSampleToObservation1p5.getEndTime(), //releases the sample
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //T,G, and P the 2nd sample
        LinearElbow elbowSampleToObservation2 = new LinearElbow(clawSampleToObservation1p5.getStartTime(), //prepares to go down to sample
                new ElbowState(elbowDown),
                new ElbowState(elbowPartialUp)
        );

//		System.out.println(clawSampleToObservation1p5.getStartTime());
//		System.out.println(clawSampleToObservation1p5.getEndTime());
//
//		System.out.println(splineSampleToObservation1p5.getEndTime());

        LinearRotation rotateSampleToObservation2 = new LinearRotation(clawSampleToObservation1p5.getStartTime(), //rotates to sample
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowSampleToObservation2p5 = new LinearElbow(splineSampleToObservation1p5.getEndTime() , //goes down to sample
                new ElbowState(elbowPartialUp),
                new ElbowState(elbowDown)
        );

//		System.out.println(rotateSampleToObservation2.getEndTime());

        LinearClaw clawSampleToObservation2 = new LinearClaw(elbowSampleToObservation2p5.getEndTime(), //closes claw
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        CRSplineTranslation splineSampleToObservation2 = new CRSplineTranslation(clawSampleToObservation2.getEndTime(), //moves to observation and then to third sample
                new TranslationState(-47.5, 38.5),
                new TranslationState(-52.75, 49),
                new TranslationState(-53, 44),
                new TranslationState(-57.5, 38.5)
        );

        LinearRotation rotateSampleToObservation2p5 = new LinearRotation(clawSampleToObservation2.getEndTime(), //rotates to observation
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearClaw clawSampleToObservation2p5 = new LinearClaw(rotateSampleToObservation2p5.getEndTime(), //releases sample
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //T,G, and P the 3rd sample
        LinearElbow elbowSampleToObservation3 = new LinearElbow(clawSampleToObservation2p5.getStartTime(), //prepares to go down to sample
                new ElbowState(elbowDown),
                new ElbowState(elbowPartialUp)
        );

        LinearRotation rotateSampleToObservation3 = new LinearRotation(clawSampleToObservation2p5.getStartTime(), //rotates to sample
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowSampleToObservation3p5 = new LinearElbow(rotateSampleToObservation3.getEndTime(), //goes down to sample
                new ElbowState(elbowPartialUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawSampleToObservation3 = new LinearClaw(elbowSampleToObservation3p5.getEndTime(), //closes claw
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        CRSplineTranslation splineSampleToObservation3 = new CRSplineTranslation(clawSampleToObservation3.getEndTime(), //moves to observation
                new TranslationState(-57.5, 38.5),
                new TranslationState(-53, 44),
                new TranslationState(-52.75, 49)
        );

        LinearRotation rotateSampleToObservation3p5 = new LinearRotation(clawSampleToObservation3.getEndTime(), //rotates to observation
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearClaw clawSampleToObservation3p5 = new LinearClaw(splineSampleToObservation3.getEndTime(), //releases sample
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ///////////////////////////////////////////////////////////////////(STEP 3)
        //// leave observation zone
        CRSplineTranslation leaveThenEnterObservationTranslation = new CRSplineTranslation(clawSampleToObservation3p5.getEndTime(), //leaves observation to reorient
                new TranslationState(-52.75, 49),
                new TranslationState( -50, 47),
                new TranslationState(-48, 40)
        );

        LinearRotation leaveThenEnterObservationRotate = new LinearRotation(clawSampleToObservation3p5.getEndTime(), //rotates to face specimen
                new RotationState(Math.toRadians(141)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownObservationSetUp = new LinearElbow(clawSampleToObservation3p5.getEndTime(), //raises elbow slightly
                new ElbowState(elbowDown),
                new ElbowState(elbowPartialUp)
        );

        ///////////////////////////////////////////////////////////////////(STEP 4)
        //// Pick up from observation zone #1
//		LinearTranslation lineEnterObservation = new LinearTranslation(leaveThenEnterObservationTranslation.getEndTime(), //moves towards specimen
//				new TranslationState(-48, 45),
//				new TranslationState(-48, 40)
//		);

        LinearElbow elbowDownObservation1 = new LinearElbow(leaveThenEnterObservationTranslation.getEndTime() - 0.25, //goes down to specimen
                new ElbowState(elbowPartialUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseObservation1 = new LinearClaw(elbowDownObservation1.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowUpObservation1 = new LinearElbow(clawCloseObservation1.getEndTime()+0.11,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ///////////////////////////////////////////////////////////////////(STEP 5)
        //// Deposit specimen(Human-Player) at submersible
        CRSplineTranslation splineObservationToSubmersible1 = new CRSplineTranslation(clawCloseObservation1.getEndTime(), //move towards submersible
                new TranslationState(-48, 40),
                new TranslationState(-10, 37),
                new TranslationState(-8, 37),
                new TranslationState(-10, 37)
        );


        LinearRotation rotateToSubmersible1 = new LinearRotation(splineObservationToSubmersible1.getStartTime()+0.1, //rotates to submersible
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftToSubmersible1 = new LinearLift(splineObservationToSubmersible1.getStartTime(), //raises lift to prepare to place specimen
                new LiftState(0),
                new LiftState(1400)
        );

        LinearLift liftDownFromSubmersible1 = new LinearLift(splineObservationToSubmersible1.getEndTime()-0.5, //brings lift down to place specimen
                new LiftState(1400),
                new LiftState(0)
        );

        // claw
        LinearClaw clawSubmersibleOpen1 = new LinearClaw(liftDownFromSubmersible1.getStartTime()+.66, //opens claw to release specimen
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ///////////////////////////////////////////////////////////////////(STEP 6)
        //// pick up specimen(number 1) at observation zone
        CRSplineTranslation splineCycleToObservation1 = new CRSplineTranslation(liftDownFromSubmersible1.getEndTime(),
                new TranslationState(-10, 37),
                //new TranslationState(-24, 44),
                new TranslationState(-48, 40)
        );

        LinearRotation rotateCycleToObservation1 = new LinearRotation(splineCycleToObservation1.getStartTime()+0.1,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowCycleDownObservation1 = new LinearElbow(splineCycleToObservation1.getEndTime()  - 0.25, //goes down to specimen
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCycleCloseObservation1 = new LinearClaw(elbowCycleDownObservation1.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowCycleUpObservation1 = new LinearElbow(clawCycleCloseObservation1.getEndTime()+0.11,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );



        ///////////////////////////////////////////////////////////////////(STEP 7)
        //// Deposit specimen(number 1) at submersible

        CRSplineTranslation splineCycleObservationToSubmersible1 = new CRSplineTranslation(clawCycleCloseObservation1.getEndTime(),
                new TranslationState(-48, 40),
                new TranslationState(-10, 37),
                new TranslationState(-8, 37),
                new TranslationState(-10, 37)


        );

        LinearRotation rotateCycleToSubmersible1 = new LinearRotation(splineCycleObservationToSubmersible1.getStartTime()+0.1,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftCycle2Up1 = new LinearLift(splineCycleObservationToSubmersible1.getStartTime(),
                new LiftState(0),
                new LiftState(1400)
        );

        LinearLift liftCycle2Down1 = new LinearLift(splineCycleObservationToSubmersible1.getEndTime()-0.5,
                new LiftState(1400),
                new LiftState(0)
        );

        // claw
        LinearClaw clawCycle2Open1 = new LinearClaw(liftCycle2Down1.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ///////////////////////////////////////////////////////////////////(STEP 8)
        //// pick up specimen(number 2) at observation zone
        CRSplineTranslation splineCycleToObservation2 = new CRSplineTranslation(liftCycle2Down1.getEndTime(),
                new TranslationState(-10, 37),
                //new TranslationState(-24, 44),
                new TranslationState(-48, 40)
        );

        LinearRotation rotateCycleToObservation2 = new LinearRotation(clawCycle2Open1.getEndTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowCycleDownObservation2 = new LinearElbow(splineCycleToObservation2.getEndTime() - 0.25, //goes down to specimen
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCycleCloseObservation2 = new LinearClaw(elbowCycleDownObservation2.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowCycleUpObservation2 = new LinearElbow(clawCycleCloseObservation2.getEndTime()+0.11,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );



        ///////////////////////////////////////////////////////////////////(STEP 9)
        //// Deposit specimen(number 2) at submersible
        CRSplineTranslation splineCycleObservationToSubmersible2 = new CRSplineTranslation(clawCycleCloseObservation2.getEndTime(),
                new TranslationState(-48, 40),
                new TranslationState(-10, 37),
                new TranslationState(-8, 37),
                new TranslationState(-10, 37)


        );

        LinearRotation rotateCycleToSubmersible2 = new LinearRotation(splineCycleObservationToSubmersible2.getStartTime()+0.1,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftCycle2Up2 = new LinearLift(splineCycleObservationToSubmersible2.getStartTime(),
                new LiftState(0),
                new LiftState(1400)
        );

        LinearLift liftCycle2Down2 = new LinearLift(splineCycleObservationToSubmersible2.getEndTime()-0.5,
                new LiftState(1400),
                new LiftState(0)
        );

        // claw
        LinearClaw clawCycle2Open2 = new LinearClaw(liftCycle2Down2.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );


        ///////////////////////////////////////////////////////////////////(STEP ???????????????)
        //// Park in observation zone
        CRSplineTranslation splinePark = new CRSplineTranslation(liftCycle2Down2.getEndTime(), //goes to observation zone
                new TranslationState(-10, 37),
                //new TranslationState(-18, 48),
                new TranslationState(-35, 60)
        );





        // Create Plans

        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                splineSampleToObservation1,
                splineSampleToObservation1p5,
                splineSampleToObservation2,
                splineSampleToObservation3,
                leaveThenEnterObservationTranslation,
                //lineEnterObservation,
                splineObservationToSubmersible1,
                splineCycleToObservation1,
                splineCycleObservationToSubmersible1,
                splineCycleToObservation2,
                splineCycleObservationToSubmersible2,
                splinePark
                //lineLeaveObservation,
                //lineEnterObservation,
                //splineObservationToSubmersible,
                //splineCycleToObservation,
                //splineCycleObservationToSubmersible,
                //splinePark
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                still,
                rotateSampleToObservation1,
                rotateSampleToObservation1p5,
                rotateSampleToObservation2,
                rotateSampleToObservation2p5,
                rotateSampleToObservation3,
                rotateSampleToObservation3p5,
                leaveThenEnterObservationRotate,
                rotateToSubmersible1,
                rotateCycleToObservation1,
                rotateCycleToSubmersible1,
                rotateCycleToObservation2,
                rotateCycleToSubmersible2
                //rotateToObservation,
                //rotateToSubmersible,
                //rotateCycleToObservation,
                //rotateCycleToSubmersible
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftToSubmersible1,
                liftDownFromSubmersible1,
                liftCycle2Up1,
                liftCycle2Down1,
                liftCycle2Up2,
                liftCycle2Down2
//				liftCycleUp,
//				liftCycleDown,
//				liftCycle2Up,
//				liftCycle2Down
        );

        ClawPlan clawPlan = new ClawPlan(robot,
                claw1,
                clawSampleToObservation1,
                clawSampleToObservation1p5,
                clawSampleToObservation2,
                clawSampleToObservation2p5,
                clawSampleToObservation3,
                clawSampleToObservation3p5,
                clawCloseObservation1,
                clawSubmersibleOpen1,
                clawCycleCloseObservation1,
                clawCycle2Open1,
                clawCycleCloseObservation2,
                clawCycle2Open2
//				clawCloseObservation,
//				clawCycleOpen,
//				clawCycleCloseObservation,
//				clawCycle2Open
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbowStill,
                elbowSampleToObservation1,
                elbowSampleToObservation2,
                elbowSampleToObservation2p5,
                elbowSampleToObservation3,
                elbowSampleToObservation3p5,
                elbowDownObservationSetUp,
                elbowDownObservation1,
                elbowUpObservation1,
                elbowCycleDownObservation1,
                elbowCycleUpObservation1,
                elbowCycleDownObservation2,
                elbowCycleUpObservation2
//				elbowDownObservation,
//				elbowUpObservation,
//				elbowCycleDownObservation,
//				elbowCycleUpObservation
        );

        this.synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan,
                liftPlan,
                elbowPlan,
                clawPlan
        );
    }
}
