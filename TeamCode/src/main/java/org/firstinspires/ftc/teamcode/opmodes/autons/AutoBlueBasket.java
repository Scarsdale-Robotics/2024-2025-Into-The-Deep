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
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
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

@Autonomous(name="Auto Blue Basket", group = "Autons")
public class AutoBlueBasket extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    public static double clawOpen = 1;
    public static double clawClosed = 0.91;

    public static double elbowUp = 0.275;
    public static double elbowDown = 0.52;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(40, 60, new Rotation2d(Math.toRadians(-90))), this);
        robot.inDep.setElbowPosition(0.3);
        initSynchronizer();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) {
            robot.localization.update();
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


        // place preloaded specimen

        CRSplineTranslation spline1 = new CRSplineTranslation(0,
                new TranslationState(40,60),
                new TranslationState(20, 40),
                new TranslationState(5, 35)
//				new TranslationState(40,50),
//				new TranslationState(48,30)
        );


        LinearLift liftPreload1 = new LinearLift(new TimeSpan(spline1.getStartTime(), spline1.getEndTime()),
                new LiftState(0),
                new LiftState(2000)
        );

        // go to first sample
        CRSplineTranslation spline1p5 = new CRSplineTranslation((spline1.getEndTime()),
                new TranslationState(5, 35),
                new TranslationState(40,50),
                new TranslationState(48,42)
        );

        LinearLift liftPreload2 = new LinearLift(new TimeSpan(spline1p5.getStartTime(), spline1p5.getEndTime()),
                new LiftState(2000),
                new LiftState(0)
        );

        LinearRotation rot1 = new LinearRotation(spline1p5.getEndTime()-0.5,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(45))
        );

        LinearLift liftFirstSample1 = new LinearLift(spline1p5.getEndTime(),
                new LiftState(0),
                new LiftState(4200)
        );

        CRSplineTranslation spline2 = new CRSplineTranslation(new TimeSpan(liftFirstSample1.getStartTime(), liftFirstSample1.getEndTime()),
                new TranslationState(48,42),
                new TranslationState(47,45),
                new TranslationState(50,50)
        );

        // go to second sample

        LinearRotation rot2 = new LinearRotation(spline2.getEndTime()-0.5,
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftFirstSample2 = new LinearLift(rot2.getStartTime()+0.5,
                new LiftState(4200),
                new LiftState(0)
        );

        LinearTranslation line1 = new LinearTranslation(new TimeSpan(spline2.getEndTime(), Math.max(rot2.getEndTime()+0.5, liftFirstSample2.getEndTime())),
                new TranslationState(50,50),
                new TranslationState(60,42)
        );

        LinearRotation rot3 = new LinearRotation(line1.getEndTime()-0.5,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(45))
        );

        LinearLift liftSecondSample1 = new LinearLift(line1.getEndTime(),
                new LiftState(0),
                new LiftState(4200)
        );

        CRSplineTranslation spline3 = new CRSplineTranslation(new TimeSpan(liftSecondSample1.getStartTime(), liftSecondSample1.getEndTime()),
                new TranslationState(60,42),
                new TranslationState(55,43),
                new TranslationState(50,50)
        );

        // pick up third sample

        LinearLift liftSecondSample2 = new LinearLift(spline3.getEndTime(),
                new LiftState(4200),
                new LiftState(0)
        );

        CRSplineTranslation spline4 = new CRSplineTranslation(new TimeSpan(liftSecondSample2.getStartTime(), liftSecondSample2.getEndTime()),
                new TranslationState(50,50),
                new TranslationState(45,36),
                new TranslationState(60,24)
        );

        LinearLift liftThirdSample1 = new LinearLift(spline4.getEndTime(),
                new LiftState(0),
                new LiftState(4200)
        );

        CRSplineTranslation spline5 = new CRSplineTranslation(new TimeSpan(liftThirdSample1.getStartTime(), liftThirdSample1.getEndTime()),
                new TranslationState(60,24),
                new TranslationState(54,43),
                new TranslationState(50,50)
        );



        // park
        CRSplineTranslation spline6 = new CRSplineTranslation(spline5.getEndTime(),
                new TranslationState(50,50),
                new TranslationState(55, 10),
                new TranslationState(25,10)
        );

        LinearLift liftThirdSample2 = new LinearLift(new TimeSpan(spline6.getStartTime(), spline6.getEndTime()),
                new LiftState(4200),
                new LiftState(0)
        );

        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                spline1p5,
                spline2,
                line1,
                spline3,
                spline4,
                spline5,
                spline6
        );




        LinearRotation rot4 = new LinearRotation(spline4.getStartTime()-0.5,
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(0))
        );

        LinearRotation rot5 = new LinearRotation(spline5.getStartTime()-0.5,
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(45))
        );

        LinearRotation rot6 = new LinearRotation(spline6.getStartTime()-0.5,
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(180))
        );


        RotationPlan rotationPlan = new RotationPlan(robot,
                rot1,
                rot2,
                rot3,
                rot4,
                rot5,
                rot6
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftFirstSample1,
                liftFirstSample2,
                liftSecondSample1,
                liftSecondSample2,
                liftThirdSample1,
                liftThirdSample2
        );

        // claw
        LinearClaw claw1 = new LinearClaw(liftPreload2.getStartTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearClaw claw2 = new LinearClaw(liftFirstSample1.getStartTime(), // grabs sample 1
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearClaw claw3 = new LinearClaw(liftFirstSample2.getStartTime(), // puts sample 1 into basket
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearClaw claw4 = new LinearClaw(liftSecondSample1.getStartTime(), // grabs sample 2
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearClaw claw5 = new LinearClaw(liftSecondSample2.getStartTime(), // puts sample 2 into basket
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearClaw claw6 = new LinearClaw(liftThirdSample1.getStartTime(), // grabs sample 3
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearClaw claw7 = new LinearClaw(liftThirdSample2.getStartTime(), // puts sample 3 into basket
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ClawPlan clawPlan = new ClawPlan(robot,
                claw1,
                claw2,
                claw3,
                claw4,
                claw5,
                claw6,
                claw7
        );

        //elbow
        LinearElbow elbow1 = new LinearElbow(claw1.getStartTime(), //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearElbow elbow2 = new LinearElbow(claw2.getStartTime(), //goes up
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        LinearElbow elbow3 = new LinearElbow(claw4.getStartTime(), //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow4 = new LinearElbow(claw5.getStartTime(), //goes up
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        LinearElbow elbow5 = new LinearElbow(claw6.getStartTime(), //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow6 = new LinearElbow(claw7.getStartTime(), //goes up
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbow1,
                elbow2,
                elbow3,
                elbow4,
                elbow5,
                elbow6
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
