package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;

public class RobotSystem {

    public final LinearOpMode opMode;
    public final Telemetry telemetry;

    public final DriveSubsystem drive;
    public final CVSubsystem cv;
    public final LocalizationSubsystem localization;
    public final InDepSubsystem inDep;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

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
                cv,
                hardwareRobot.pinpoint,
                opMode
                ,telemetry
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

        this.inDep.setClawPosition(clawClosed);
        this.inDep.setElbowPosition(elbowUp-0.04);

        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH, initialPose.getX(), initialPose.getY(), AngleUnit.RADIANS, initialPose.getHeading());
        while (opMode.opModeInInit()) {
            this.localization.pinpoint.update();
            this.localization.pinpoint.setPosition(initialPose2D);

            this.telemetry.addData("PP X after robotsys", this.localization.pinpoint.getPosX());
            this.telemetry.addData("PP Y after robotsys", this.localization.pinpoint.getPosY());
            this.telemetry.addData("PP heading after robotsys", this.localization.pinpoint.getHeading());
            this.telemetry.addData("odom X after robotsys", this.localization.getX());
            this.telemetry.addData("odom Y after robotsys", this.localization.getY());
            this.telemetry.addData("odom heading after robotsys", this.localization.getH());
            this.telemetry.update();
        }
    }

    public void logOdometry() {
        Pose2d currentPose = localization.getPose();
        telemetry.addData("[ODO] X", currentPose.getX());
        telemetry.addData("[ODO] Y", currentPose.getY());
        telemetry.addData("[ODO] H", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), currentPose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
