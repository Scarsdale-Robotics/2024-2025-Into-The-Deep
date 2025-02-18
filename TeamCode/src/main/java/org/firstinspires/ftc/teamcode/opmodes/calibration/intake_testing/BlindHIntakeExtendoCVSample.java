package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.GrabHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;

import java.util.ArrayDeque;
import java.util.ArrayList;

@Config
@Autonomous(name="Blind Search For Sample Using Horizontal Intake, Extendo, and CV (no drivetrain)", group = "Calibration")
public class BlindHIntakeExtendoCVSample extends LinearOpMode {

    private Synchronizer search, pickup;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;

    public static double armDownPosition = 1.025;
    public static double timeBuffer = 0.045;
    private ArrayList<double[]> bufferedExtendoPositions;  // [{position, timestamp}, ...]
    private double lastBufferedExtendoPosition;

    public static double searchSpeedDivisor = 6;
    public static double pickupSpeedDivisor = 8;
    public static double clawGrabClipTime = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        initSearch();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        bufferedExtendoPositions = new ArrayList<>();
        bufferedExtendoPositions.add(new double[] {
                linearSlides.getExtendoPosition(),
                runtime.seconds()
        });
        lastBufferedExtendoPosition = bufferedExtendoPositions.get(0)[0];

        waitForStart();
        overheadCamera.correctExposure(this, telemetry);

        while (opModeIsActive()) {
            // Wait for button press
            while (opModeIsActive() && gamepad1.square) updateTPS();
            while (opModeIsActive() && !gamepad1.square) updateTPS();

            // Search motion
            search.start();
            ExtendoState position = null;
            ExtendoState velocity = null;
            double[] closestSample = null;
            while (opModeIsActive() && search.update()) {
                updateTPS();
                closestSample = overheadCamera.getClosestSample();
                if (closestSample != null) {
                    position = new ExtendoState(lastBufferedExtendoPosition);
                    velocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                    break;
                }
            }
            if (velocity == null) {
                // no samples found
                search.stop();
                continue;
            }

            // do not stop the synchronizer, we're going to use a dynamic movement
            initPickUp(closestSample, position, velocity);

            // Pickup motion
            pickup.start();
            while (opModeIsActive() && pickup.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.square) {
                pickup.update();
                updateTPS();
            }
            pickup.stop();
            search.stop();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // init horizontal intake
        Servo leftHorizontalArm = hardwareMap.get(ServoImplEx.class, "leftHorizontalArm");
        Servo rightHorizontalArm = hardwareMap.get(ServoImplEx.class, "rightHorizontalArm");
        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");
        Servo horizontalClaw = hardwareMap.get(ServoImplEx.class, "horizontalClaw");
        this.horizontalIntake = new HorizontalIntakeSubsystem(
                leftHorizontalArm,
                rightHorizontalArm,
                horizontalWrist,
                horizontalClaw
        );
        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.9);

        // init overhead camera
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.overheadCamera = new OverheadCameraSubsystem(cameraName, telemetry);

        // init linear slides
        Motor extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620);
        Motor leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        Motor rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(true);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(false);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);

        this.linearSlides = new LinearSlidesSubsystem(extendo, leftLift, rightLift, telemetry);
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        linearSlides.update();
        telemetry.update();

        double currentExtendoPosition = linearSlides.getExtendoPosition();
        bufferedExtendoPositions.add(new double[]{
                currentExtendoPosition,
                runtime.seconds()
        });
        while (currentTime - bufferedExtendoPositions.get(0)[1] > timeBuffer) bufferedExtendoPositions.remove(0);
        lastBufferedExtendoPosition = bufferedExtendoPositions.get(0)[0];
    }

    private void initSearch() {
        double currentExtendoPosition = linearSlides.getExtendoPosition();
        double extendoTarget = 15;

        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / searchSpeedDivisor;
        // Extend and retract
        LinearExtendo extendoOut = new LinearExtendo(0,
                new ExtendoState(currentExtendoPosition),
                new ExtendoState(extendoTarget)
        );
        LinearExtendo extendoIn = new LinearExtendo(extendoOut.getEndTime(),
                new ExtendoState(extendoTarget),
                new ExtendoState(0)
        );
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

        // Keep other subsystems still
        MoveHWrist h_wrist_reset = new MoveHWrist(0, 0);
        ReleaseHClaw h_claw_release = new ReleaseHClaw(0);
        LinearHArm h_arm_up = new LinearHArm(0,
                new HArmState(0.9),
                new HArmState(0.9)
        );

        // Create Plans
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoOut,
                extendoIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_up
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_release
        );

        // Synchronizer
        this.search = new Synchronizer(
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }

    private void initPickUp(double[] samplePosition, ExtendoState position, ExtendoState velocity) {
        double x = samplePosition[0];
        double y = samplePosition[1];
        double angle = samplePosition[2];

        double currentExtendoPosition = position.getLength();
        double extendoTarget = currentExtendoPosition + x - OverheadCameraSubsystem.CLAW_OFFSET[0];
        extendoTarget = Math.max(0, extendoTarget);

        // Extendo
        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / pickupSpeedDivisor;
        DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                new ExtendoState(currentExtendoPosition),
                new ExtendoState(extendoTarget),
                velocity
        );
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(extendoOut.getEndTime(),
                new HArmState(0.9),
                new HArmState(armDownPosition)
        );
        MoveHWrist h_wrist_align = new MoveHWrist(h_arm_down.getStartTime(), angle);

        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime()-clawGrabClipTime);
        MoveHWrist h_wrist_reset = new MoveHWrist(h_claw_grab.getEndTime()+clawGrabClipTime, 0);
        LinearHArm h_arm_up = new LinearHArm(h_wrist_reset.getEndTime(),
                new HArmState(armDownPosition),
                new HArmState(0.9)
        );

        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(h_arm_up.getStartTime(),
                new ExtendoState(extendoTarget),
                new ExtendoState(0)
        );

        // Create Plans
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoOut,
                extendoIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_grab
        );

        // Synchronizer
        this.pickup = new Synchronizer(
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }
}
