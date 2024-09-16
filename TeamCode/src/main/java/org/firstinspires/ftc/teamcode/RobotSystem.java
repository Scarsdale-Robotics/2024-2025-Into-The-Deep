package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

public class RobotSystem {

    public final OpMode opMode;
    public final Telemetry telemetry;

    public final DriveSubsystem drive;
    public final CVSubsystem cv;
    public final LocalizationSubsystem localization;

    public RobotSystem(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
        this.cv = new CVSubsystem(
                hardwareRobot.limelight,
                telemetry
        );
        this.localization = new LocalizationSubsystem(
                new Pose2d(0, 0, new Rotation2d(0)),
                hardwareRobot.leftOdometer,
                hardwareRobot.rightOdometer,
                hardwareRobot.centerOdometer,
                cv,
                telemetry
        );
        this.drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack
        );

    }

}
