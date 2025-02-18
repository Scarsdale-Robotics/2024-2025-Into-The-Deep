package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initPinpoint;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;
import static org.firstinspires.ftc.teamcode.Auxiliary.initWebcam;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakePlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.LinearMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.MoveMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;

import java.util.function.Supplier;

@Config
@Autonomous(name="Clip Intake Motion TWOOO", group = "Calibration")
public class ClipIntakeMotionTwooo extends LinearOpMode {

    private boolean lastAutoIntakeCommand = false;

    private Synchronizer autoIntake;

    private Synchronizer[] manualIntakeSynchronizer;
    private Synchronizer clipIntakeSynchronizer;

    private double wristPos = 0;
    private double extendoPos = 0;

    private static final int MAX_WRIST = 1;
    private static final int MIN_WRIST = 0;

    private static final double EXTENDO_SPEED = 3.0;

    private static final int MAX_EXTENDO = 2000;
    private static final int MIN_EXTENDO = 200;  // pos must be safe to flip arm

    private static final double TRANSFER_ARM = 0.2;
    private static final double TRANSFER_WRIST = 0;
    private static final double APPROACH_ARM = 0.95;
    private static final double INTAKE_ARM = 1.05;
    private static final int TRANSFER_EXTENDO = 200;

    public static double M_INTAKE_OPEN = 0.5;
    public static double M_INTAKE_UP = 0.8;
    public static double M_INTAKE_CLOSED = 0.21;
    public static double M_LOADER_OPEN = 0.86;
    public static double M_LOADER_CLOSED = 0.1;
    public static int M_FEEDER_MAX = 10000;
    public static int M_FEEDER_MIN = 0;

    private static boolean RUN_INTAKE = true;
    private static boolean RUN_CLIPBOT = true;

    private LinearSlidesSubsystem linearSlides;
    private HorizontalIntakeSubsystem horizontalIntake;
    private ClipbotSubsystem clipbot;
    private OverheadCameraSubsystem overheadCamera;
    private DriveSubsystem drive;
    private LocalizationSubsystem localization;
    private GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean autoIntakingEnabled = true;
        int intakeIndex = 0;

//        Servo horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");

        // NO MOVEMENT BEFORE START

        clipbot = new ClipbotSubsystem(
                initServo(hardwareMap, "magazineIntake"),
                initServo(hardwareMap, "magazineLoader1"),
                initServo(hardwareMap, "magazineLoader2"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_312),
                hardwareMap.dcMotor.get("magazineFeeder")
        );

        waitForStart();

        runtime = new ElapsedTime(0);
        runtime.reset();

        boolean clipIntakeRunning = false;
        while (opModeIsActive()) {
            if (!clipIntakeRunning) {
                updateSyncs();
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                clipIntakeSynchronizer.start();
                clipIntakeRunning = true;
            }

            if (clipIntakeRunning) {
                clipIntakeRunning = clipIntakeSynchronizer.update();
            }
        }
    }

    private void updateSyncs() {

        Supplier<Synchronizer> TEST_NO_TRANSLATION_clipIntake = () -> {
            MoveMIntake intakeOpen = new MoveMIntake(0, M_INTAKE_OPEN);
            MoveMLoader loaderOpen = new MoveMLoader(0, M_LOADER_OPEN);

            MoveMIntake intakeUp = new MoveMIntake(
                    Math.max(
                            intakeOpen.getEndTime(),
                            loaderOpen.getEndTime()
                    ) + 5,
                    M_INTAKE_UP
            );

            LinearMIntake intakeClose = new LinearMIntake(
                    new TimeSpan(
                            intakeUp.getEndTime() + 5,
                            intakeUp.getEndTime() + 15
                    ),
                    new MIntakeState(M_INTAKE_UP),
                    new MIntakeState(M_INTAKE_CLOSED)
            );
            MoveMLoader loaderClose = new MoveMLoader(
                    intakeClose.getEndTime(),
                    M_LOADER_CLOSED
            );

            MIntakePlan intakePlan = new MIntakePlan(
                    clipbot,
                    intakeOpen, intakeUp, intakeClose
            );
            MLoaderPlan loaderPlan = new MLoaderPlan(
                    clipbot,
                    loaderOpen, loaderClose
            );

            return new Synchronizer(
                    intakePlan,
                    loaderPlan
            );
        };

        clipIntakeSynchronizer = TEST_NO_TRANSLATION_clipIntake.get();
    }
}
