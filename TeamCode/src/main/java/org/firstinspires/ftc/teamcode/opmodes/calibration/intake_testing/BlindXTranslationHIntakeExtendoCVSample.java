package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.ArrayDeque;
import java.util.ArrayList;

@Config
@Autonomous(name="Blind Pick Up Sample Using Horizontal Intake, Extendo, and Drivebase Translation along the X", group = "Calibration")
public class BlindXTranslationHIntakeExtendoCVSample extends LinearOpMode {

    private Synchronizer search, pickup;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    public static double timeBuffer = 0.045;
    private ArrayList<double[]> bufferedExtendoPositions;  // [{position, timestamp}, ...]
    private ArrayList<Pose2d> bufferedBotPoses;
    private double lastBufferedExtendoPosition;
    private Pose2d lastBufferedBotPose;

    public static double armDownPosition = 1.025;
    public static double searchSpeedDivisor = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems(new Pose2d(0,0,new Rotation2d(Math.toRadians(90))));
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
        bufferedBotPoses = new ArrayList<>();
        bufferedBotPoses.add(
                localization.getPose()
        );
        lastBufferedBotPose = bufferedBotPoses.get(0);

        waitForStart();
        overheadCamera.correctExposure();

        while (opModeIsActive()) {
            // Create synchronizer on button press
            horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
            while (opModeIsActive() && gamepad1.square) updateTPS();
            while (opModeIsActive() && !gamepad1.square) updateTPS();

            // Search motion
            search.start();
            ExtendoState extendoPosition = null;
            ExtendoState extendoVelocity = null;
            Pose2d botPose = null;
            double[] closestSample = null;
            while (opModeIsActive() && search.update()) {
                updateTPS();
                closestSample = overheadCamera.getClosestSample();
                if (closestSample != null) {
                    extendoPosition = new ExtendoState(lastBufferedExtendoPosition);
                    extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                    botPose = lastBufferedBotPose;
                    break;
                }
            }
            if (extendoVelocity == null) {
                // no samples found
                search.stop();
                continue;
            }

            // do not stop the synchronizer, we're going to use a dynamic movement
            initPickup(closestSample, botPose, extendoPosition, extendoVelocity);

            // Run synchronizer
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

    private void initSubsystems(Pose2d initialPose) {
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
        // TODO: FIGURE OUT WHAT RPM EXTENDO MOTOR IS
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

        // init localization
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.localization = new LocalizationSubsystem(
                initialPose,
                pinpoint,
                this,
                telemetry
        );

        // init drive
        MotorEx leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        MotorEx rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        MotorEx leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        MotorEx rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );


        //// init initial pose
        telemetry.addData("status", "setting pinpoint initialPose");
        telemetry.update();

        // wait for pinpoint to get ready
        while (opModeIsActive() && pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY);

        // create comparators
        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH, initialPose.getX(), initialPose.getY(), AngleUnit.RADIANS, initialPose.getHeading());
        double initialX = initialPose.getX();
        double initialY = initialPose.getY();
        double initialH = initialPose.getHeading();
        Pose2D ppPose = pinpoint.getPosition();
        double ppX = ppPose.getX(DistanceUnit.INCH);
        double ppY = ppPose.getY(DistanceUnit.INCH);
        double ppH = ppPose.getHeading(AngleUnit.RADIANS);

        // wait until initial position has been set
        while (!(equal(ppX,initialX) && equal(ppY,initialY) && equal(ppH,initialH))) {
            pinpoint.setPosition(initialPose2D);
            pinpoint.update();
            ppPose = pinpoint.getPosition();
            ppX = ppPose.getX(DistanceUnit.INCH);
            ppY = ppPose.getY(DistanceUnit.INCH);
            ppH = ppPose.getHeading(AngleUnit.RADIANS);

            // Telemetry
            telemetry.addData("status", "setting pinpoint initialPose");
            telemetry.addData("(status) ppX", ppX);
            telemetry.addData("(status) ppY", ppY);
            telemetry.addData("(status) ppH", ppH);
            telemetry.update();
        }
        telemetry.addData("status", "finished init");
        telemetry.update();
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        localization.update();
        telemetry.update();

        //// camera lag buffering
        // extendo
        double currentExtendoPosition = linearSlides.getExtendoPosition();
        bufferedExtendoPositions.add(new double[]{
                currentExtendoPosition,
                runtime.seconds()
        });
        // botpose
        Pose2d currentBotPose = localization.getPose();
        bufferedBotPoses.add(
                currentBotPose
        );
        while (currentTime - bufferedExtendoPositions.get(0)[1] > timeBuffer) {
            bufferedExtendoPositions.remove(0);
            bufferedBotPoses.remove(0);
        }
        lastBufferedExtendoPosition = bufferedExtendoPositions.get(0)[0];
        lastBufferedBotPose = bufferedBotPoses.get(0);
    }

    private void initSearch() {
        double currentExtendoPosition = linearSlides.getExtendoPosition();
        double extendoTarget = 16;

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

    private void initPickup(double[] samplePosition, Pose2d botPose, ExtendoState extendoPosition, ExtendoState extendoVelocity) {
        //// Calculating real sample position

        // unpack bot pose & extendo state
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();
        double x_extendo = extendoPosition.getLength();

        // get cv readings
        double x_sample_cam = samplePosition[0];
        double y_sample_cam = samplePosition[1];
        double theta_sample_cam = samplePosition[2];

        // convert to robot frame
        double x_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[0] + x_extendo + x_sample_cam;
        double y_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[1] + y_sample_cam;
        double theta_sample_bot = Math.atan2(y_sample_bot, x_sample_bot);
        double distance_sample_bot = Math.hypot(x_sample_bot, y_sample_bot);
        double bearing = heading_bot + theta_sample_bot;

        // find real sample coordinates
        double x_sample = x_bot + distance_sample_bot * Math.cos(bearing);
        double y_sample = y_bot + distance_sample_bot * Math.sin(bearing);
        double theta_sample = (theta_sample_cam + heading_bot) - Math.PI/2;

        // get subsystem setpoints along the X
        TranslationState translationTarget;
        RotationState rotationTarget;
        if (y_sample < y_bot) {
            translationTarget = new TranslationState(x_sample-OverheadCameraSubsystem.CAMERA_OFFSET[1], y_bot);
            rotationTarget = new RotationState(-Math.PI/2);
        }
        else {
            translationTarget = new TranslationState(x_sample+OverheadCameraSubsystem.CAMERA_OFFSET[1], y_bot);
            rotationTarget = new RotationState(Math.PI/2);
        }
        ExtendoState extendoTarget = new ExtendoState(
                Math.abs(y_sample - y_bot) - (OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0])
        );
        double hWristTarget = normalizeAngle(theta_sample);


        //// SYNCHRONIZER
        // Extendo
        LinearTranslation translation = new LinearTranslation(0,
                new TranslationState(x_bot,y_bot),
                translationTarget
        );

        LinearRotation rotation = new LinearRotation(0,
                new RotationState(heading_bot),
                rotationTarget
        );

        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / 3;
        DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                extendoPosition,
                extendoTarget,
                extendoVelocity
        );
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(Math.max(Math.max(extendoOut.getEndTime(), rotation.getEndTime()), translation.getEndTime()),
                new HArmState(0.9),
                new HArmState(armDownPosition),
                true
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendoOut.getStartTime(), hWristTarget);

        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime(), true);
        LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime(),
                new HArmState(armDownPosition),
                new HArmState(0.9)
        );
        MoveHWrist h_wrist_reset = new MoveHWrist(h_arm_up.getEndTime(), 0, true);

        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(h_wrist_reset.getStartTime(),
                extendoTarget,
                new ExtendoState(0)
        );

        // Create Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                translation
        );
        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotation
        );
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
                translationPlan,
                rotationPlan,
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians >= Math.PI) radians -= 2*Math.PI;
        while (radians < -Math.PI) radians += 2*Math.PI;
        return radians;
    }

    /**
     * Clips the input x between a given lower and upper bound.
     * @param x
     * @param lower
     * @param upper
     * @return the clipped value of x.
     */
    private static double bound(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }

    /**
     * Determines whether the two inputs are approximately equal to each other
     * within an epsilon of 1e-3
     * @param a
     * @param b
     * @return Math.abs(a-b) <= 1e-3
     */
    private static boolean equal(double a, double b) {
        return Math.abs(a-b) <= 1e-3;
    }

}
