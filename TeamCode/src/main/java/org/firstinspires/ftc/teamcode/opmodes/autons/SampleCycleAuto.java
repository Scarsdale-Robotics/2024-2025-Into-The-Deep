package org.firstinspires.ftc.teamcode.opmodes.autons;

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
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.ArrayDeque;

@Config
@Autonomous(name="Sample Cycle Auto (Net Zone)", group = "Calibration")
public class SampleCycleAuto extends LinearOpMode {

    private Synchronizer pickup;
    private Synchronizer score;
    private Synchronizer search;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    public static double timeClip = 1;
    public static double armDownPosition = 1.025;

    public static double timeBuffer = 0.11;
    private double[] lastBufferedExtendoPosition;  // {position, timestamp}

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems(new Pose2d(40, 63.5, new Rotation2d(Math.toRadians(-90))));

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        lastBufferedExtendoPosition = new double[] {
                linearSlides.getExtendoPosition(),
                runtime.seconds()
        };

        waitForStart();
        overheadCamera.correctExposure();

        // Score preloaded sample
        Pose2d currentPose = localization.getPose();
        double currentExtendo = linearSlides.getExtendoPosition();
        initScoreAction(
                new TranslationState(currentPose.getX(), currentPose.getY()),
                new RotationState(currentPose.getHeading()),
                new ExtendoState(currentExtendo),
                false
        );
        score.start();
        while (opModeIsActive() && score.update()) updateTPS();

        // Score tape mark samples
        for (int i = 0; i < 2 && opModeIsActive(); i++) {
            TranslationState targetTranslation = new TranslationState(37+10*i,48);
            RotationState targetRotation = new RotationState(Math.toRadians(-60));
            scoreLoop(null, targetTranslation, targetRotation, true);
        }

        // Cycle submersible
        int sampleCount = 0;
        while (opModeIsActive()) {
            TranslationState targetTranslation = new TranslationState(30, 8 - 2 * sampleCount);
            RotationState targetRotation = new RotationState(Math.toRadians(180));
            TranslationState anchorTranslation = new TranslationState(48, 18);
            scoreLoop(anchorTranslation, targetTranslation, targetRotation, false);
            sampleCount++;
        }
    }

    private void scoreLoop(TranslationState anchor, TranslationState targetTranslation, RotationState targetRotation, boolean noAnchor) {
        // init search
        Pose2d currentPose = localization.getPose();
        double currentExtendo = linearSlides.getExtendoPosition();
        initSearchAction(
                new TranslationState(currentPose.getX(), currentPose.getY()),
                targetTranslation,
                noAnchor ? targetTranslation : anchor,
                new RotationState(currentPose.getHeading()),
                targetRotation,
                new ExtendoState(currentExtendo)
        );

        // run search
        ExtendoState position = null;
        ExtendoState velocity = null;
        double[] closestSample = null;
        while (opModeIsActive() && search.update()) {
            updateTPS();
            closestSample = overheadCamera.getClosestSample();
            if (closestSample != null) {
                position = new ExtendoState(lastBufferedExtendoPosition[0]);
                velocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                break;
            }
        }

        // init pickup
        currentPose = localization.getPose();
        initPickupAction(
                closestSample,
                new TranslationState(currentPose.getX(), currentPose.getY()),
                new RotationState(currentPose.getHeading()),
                position,
                velocity
        );

        // Pickup motion
        pickup.start();
        while (opModeIsActive() && pickup.update()) updateTPS();

        // Score sample
        currentPose = localization.getPose();
        currentExtendo = linearSlides.getExtendoPosition();
        initScoreAction(
                new TranslationState(currentPose.getX(), currentPose.getY()),
                new RotationState(currentPose.getHeading()),
                new ExtendoState(currentExtendo),
                false
        );
        score.start();
        while (opModeIsActive() && score.update()) updateTPS();
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
        horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
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

        if (currentTime - lastBufferedExtendoPosition[1] > timeBuffer) {
            lastBufferedExtendoPosition = new double[] {
                    linearSlides.getExtendoPosition(),
                    runtime.seconds()
            };
        }
    }

    private void initPickupAction(double[] samplePosition, TranslationState currentTranslation, RotationState currentRotation, ExtendoState currentExtendoPosition, ExtendoState currentExtendoVelocity) {
        TranslationConstants.MAX_VELOCITY = 0.8*40d;
        TranslationConstants.MAX_ACCELERATION = 100d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        ExtendoConstants.MAX_VELOCITY = 105/8d;

        // Calculating rotated sample position
        double x_sample_cam = samplePosition[0];
        double y_sample_cam = samplePosition[1];
        double theta_sample_cam = samplePosition[2];
        double x_extendo = currentExtendoPosition.getLength();

        // Convert to robot frame
        double x_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[0] + x_extendo + x_sample_cam;
        double y_sample_bot = y_sample_cam;
        double bearing = Math.atan2(y_sample_bot, x_sample_bot);

        // Get rotated sample coordinates
        double theta_sample_bot = theta_sample_cam - bearing;
        double distance_sample_bot = Math.hypot(x_sample_bot, y_sample_bot);

        // Calculate subsystem setpoints
        double extendoTarget = distance_sample_bot - OverheadCameraSubsystem.CAMERA_OFFSET[0] - OverheadCameraSubsystem.CLAW_OFFSET[0];
        extendoTarget = bound(extendoTarget, 0, ExtendoConstants.MAX_EXTENSION);

        double currentHeading = currentRotation.getHeading();
        double headingTarget = currentHeading + bearing;

        // Extendo
        LinearTranslation translation = new LinearTranslation(new TimeSpan(0,1),
                currentTranslation,
                currentTranslation
        );

        LinearRotation rotation = new LinearRotation(0,
                currentRotation,
                new RotationState(headingTarget)
        );

        DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                currentExtendoPosition,
                new ExtendoState(extendoTarget),
                currentExtendoVelocity
        );

        ReleaseHClaw h_claw_release = new ReleaseHClaw(0);

        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(Math.max(0,Math.max(extendoOut.getEndTime(), rotation.getEndTime())-timeClip),
                new HArmState(0.9),
                new HArmState(armDownPosition)
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendoOut.getStartTime(), theta_sample_bot);

        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime());
        MoveHWrist h_wrist_reset = new MoveHWrist(h_claw_grab.getEndTime(), 0);
        LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime(),
                new HArmState(armDownPosition),
                new HArmState(0.9)
        );

        // Create Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                translation
        );
        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotation
        );
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoOut
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
                h_claw_release,
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


    private void initScoreAction(TranslationState currentTranslation, RotationState currentRotation, ExtendoState currentExtendo, boolean retract) {
        TranslationConstants.MAX_VELOCITY = 0.8*40d;
        TranslationConstants.MAX_ACCELERATION = 100d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        ExtendoConstants.MAX_VELOCITY = 105;

        double extendoLengthModifier = OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0];
        double extendoLength = ExtendoConstants.MAX_EXTENSION + extendoLengthModifier;

        // unpacking current position and calculating net zone setpoints
        double x0 = currentTranslation.getX();
        double y0 = currentTranslation.getY();
        double x1 = 24*(360-2*x0-3*y0)/(144-x0-y0);
        double y1 = 120-x1;
        double d = Math.hypot(x1-x0,y1-y0);
        double theta = Math.atan2(y1-y0,x1-x0);

        // translation setpoint
        double d_buffer = 4;
        double d_tr = Math.max(0, d-extendoLength+d_buffer);
        TranslationState targetTranslation = new TranslationState(
                x0 + d_tr*Math.cos(theta),
                y0 + d_tr*Math.sin(theta)
        );

        // rotation setpoint
        RotationState targetRotation = new RotationState(theta);

        // extendo setpoint
        ExtendoState targetExtension = new ExtendoState(bound(d - extendoLengthModifier, 0, extendoLength));
        ExtendoState targetRetraction = retract ? currentExtendo : new ExtendoState(0);


        // Leave pickup position
        LinearTranslation lineToNetZone = new LinearTranslation(0,
                currentTranslation,
                targetTranslation
        );

        LinearRotation rotateToNetZone = new LinearRotation(0,
                currentRotation,
                targetRotation
        );

        LinearExtendo extendoRetract = new LinearExtendo(0,
                currentExtendo,
                targetRetraction
        );

        GrabHClaw h_claw_grab = new GrabHClaw(0);

        MoveHWrist h_wrist_reset = new MoveHWrist(0, 0);

        LinearHArm h_arm_still = new LinearHArm(0,
                new HArmState(0.9),
                new HArmState(0.9)
        );


        // Deposit sample
        LinearExtendo extendToNetZone = new LinearExtendo(
                Math.max(lineToNetZone.getEndTime(), rotateToNetZone.getEndTime()),
                targetRetraction,
                targetExtension,
                true
        );

        ReleaseHClaw h_claw_release = new ReleaseHClaw(extendToNetZone.getEndTime(), true);


        // Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                lineToNetZone
        );

        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotateToNetZone
        );

        ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides,
                extendoRetract,
                extendToNetZone
        );

        HArmPlan hArmPlan = new HArmPlan(horizontalIntake,
                h_arm_still
        );

        HWristPlan hWristPlan = new HWristPlan(horizontalIntake,
                h_wrist_reset
        );

        HClawPlan hClawPlan = new HClawPlan(horizontalIntake,
                h_claw_grab,
                h_claw_release
        );

        this.score = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                hArmPlan,
                hWristPlan,
                hClawPlan
        );
    }


    public void initSearchAction(TranslationState currentTranslation, TranslationState anchorTranslation, TranslationState targetTranslation, RotationState currentRotation, RotationState targetRotation, ExtendoState currentExtendo) {

        TranslationConstants.MAX_VELOCITY = 0.8*40d;
        TranslationConstants.MAX_ACCELERATION = 100d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        ExtendoConstants.MAX_VELOCITY = 105/6d;

        // Move to target

        CRSplineTranslation splineApproach = new CRSplineTranslation(0,
                currentTranslation,
                anchorTranslation,
                targetTranslation
        );

        LinearRotation rotationApproach = new LinearRotation(0,
                currentRotation,
                targetRotation
        );

        double extensionLimit = ExtendoConstants.MAX_EXTENSION - 6;
        LinearExtendo extendoSearch = new LinearExtendo(Math.max(splineApproach.getEndTime(), rotationApproach.getEndTime()),
                currentExtendo,
                new ExtendoState(extensionLimit)
        );


        // Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                splineApproach
        );

        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotationApproach
        );

        ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides,
                extendoSearch
        );

        HArmPlan hArmPlan = new HArmPlan(horizontalIntake,
                new LinearHArm(0, new HArmState(0.9), new HArmState(0.9))
        );

        HWristPlan hWristPlan = new HWristPlan(horizontalIntake,
                new MoveHWrist(0, 0)
        );

        HClawPlan hClawPlan = new HClawPlan(horizontalIntake,
                new ReleaseHClaw(0)
        );


        // put all the Plans into a Synchronizer
        this.search = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                hArmPlan,
                hWristPlan,
                hClawPlan
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
