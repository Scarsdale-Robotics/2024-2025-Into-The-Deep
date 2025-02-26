package org.firstinspires.ftc.teamcode.opmodes.calibration.outtake_testing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;

@TeleOp(name="Transfer From Intake Action", group="Calibration")
public class TransferFromIntakeAction extends LinearOpMode {

    private Synchronizer motion;
    private AutonomousRobot robot;

    public static double grab_pause = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(),
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );
        robot.verticalDeposit.setArmPosition(0.5);
        robot.verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
        robot.horizontalIntake.setArmPosition(0.9);
        robot.horizontalIntake.setWristAngle(0);
        robot.horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
        initSynchronizer();

        waitForStart();

        motion.start();
        while (opModeIsActive() && motion.update()) robot.update();

    }

    private void initSynchronizer() {
        // Lift goes up and extendo goes out first
        LinearExtendo extendoOut = new LinearExtendo(0,
                new ExtendoState(robot.linearSlides.getExtendoPosition()),
                new ExtendoState(4.3)
        );
        LinearLift liftUp = new LinearLift(0,
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(9.1464859)
        );

        // Horizontal arm goes up
        LinearHArm hArmUp = new LinearHArm(liftUp.getEndTime(),
                new HArmState(0.9),
                new HArmState(0.48)
        );

        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(hArmUp.getEndTime(),
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Deposit claw grabs sample
        GrabVClaw grabVClaw = new GrabVClaw(vArmDown.getEndTime() + grab_pause);

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getEndTime());

        // Deposit arm moves out of the way
        LinearVArm toClipperVArm = new LinearVArm(releaseHClaw.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(toClipperVArm.getEndTime(),
                new HArmState(0.48),
                new HArmState(0.9)
        );




        // Plans
        ExtendoPlan extendoPlan = new ExtendoPlan(robot.linearSlides,
                extendoOut
        );
        HArmPlan hArmPlan = new HArmPlan(robot.horizontalIntake,
                hArmUp,
                hArmDown
        );
        HClawPlan hClawPlan = new HClawPlan(robot.horizontalIntake,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftUp
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                vArmDown,
                toClipperVArm
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                grabVClaw
        );

        // Synchronizer
        this.motion = new Synchronizer(
                extendoPlan,
                hArmPlan,
                hClawPlan,
                liftPlan,
                vArmPlan,
                vClawPlan
        );
    }

}
