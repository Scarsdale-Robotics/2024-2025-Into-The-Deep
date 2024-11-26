package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.OdometryThread;

public class RobotSystem {

    private double TPS = 0;

    private final OdometryThread odometryThread;

    public final LinearOpMode opMode;
    public final Telemetry telemetry;

    public final DriveSubsystem drive;
    public final CVSubsystem cv;
    public final LocalizationSubsystem localization;
    public final InDepSubsystem inDep;

    public RobotSystem(HardwareMap hardwareMap, Pose2d initialPose, boolean isRedTeam, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
        this.cv = new CVSubsystem(
                hardwareRobot.limelight,
                initialPose.getHeading(),
                isRedTeam,
                telemetry);
        this.localization = new LocalizationSubsystem(
                initialPose,
                hardwareRobot.leftOdometer,
                hardwareRobot.rightOdometer,
                hardwareRobot.centerOdometer,
                cv
//                ,telemetry
        );
        this.drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack
        );
        this.inDep = new InDepSubsystem(
                hardwareRobot,
                opMode,
                drive,
                cv
        );

        // asynchronously run odometry
        telemetry.addData("TPS", TPS);
        telemetry.update();
        odometryThread = new OdometryThread(this.opMode, this.telemetry, this.localization, this);
        odometryThread.start();
    }

    public void setTPS(double TPS) {
        this.TPS = TPS;
    }

    public void logTPS() {
        telemetry.addData("TPS", TPS);
        telemetry.update();
    }
}
