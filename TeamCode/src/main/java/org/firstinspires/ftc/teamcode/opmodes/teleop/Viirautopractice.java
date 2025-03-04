package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

//files to look at:
//subsystems: sync macro
//plans and states: specimen maker
//auto and cv, teleop: spec-mkr and sync macro
//branches in general - main, specimen maker, spec-mkr, synchronizer-macro
//ALSO LOOK AT AND UNDERSTAND OLD TELEOP AND ERROR STUFF FOR MACROS RUNNING
@Autonomous (name = "Sigma autonomous for old robot")
public class Viirautopractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
      while (opModeIsActive()) {
          HardwareRobot robot = new HardwareRobot(hardwareMap);
          DriveSubsystem drive = new DriveSubsystem (
            robot.leftBack,
            robot.leftFront,
            robot.rightBack,
            robot.rightFront
          );
          MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry()));
          waitForStart();


      }
    }
}
