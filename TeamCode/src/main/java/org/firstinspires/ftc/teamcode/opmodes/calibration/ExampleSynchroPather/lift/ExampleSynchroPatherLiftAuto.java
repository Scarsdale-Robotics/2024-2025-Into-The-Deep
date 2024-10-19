package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather.lift;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Autonomous(name="Example SynchroPather Lift Auto", group = "Calibration")
public class ExampleSynchroPatherLiftAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), this);
        initSynchronizer();

        waitForStart();

        while (opModeIsActive()) {
            while (opModeIsActive() && !gamepad1.square) ;
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                robot.localization.update();
            }
            synchronizer.stop();
        }
    }


    private void initSynchronizer() {
        // Translation plan
        LinearLift lift1 = new LinearLift(0,
                new LiftState(0),
                new LiftState(3000)
        );
        LinearLift lift2 = new LinearLift(lift1.getEndTime(),
                new LiftState(3000),
                new LiftState(0)
        );
        LinearLift lift3 = new LinearLift(new TimeSpan(lift2.getEndTime(), lift2.getEndTime()+3),
                new LiftState(0),
                new LiftState(0)
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                lift1,
                lift2,
                lift3
        );


        // Synchronizer
        this.synchronizer = new Synchronizer(
                liftPlan
        );
    }

}
