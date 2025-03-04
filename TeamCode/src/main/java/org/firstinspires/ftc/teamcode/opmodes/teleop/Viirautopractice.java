package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
@Autonomous (name = "Sigma autunomous for old robot")
public class Viirautopractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
      while (opModeIsActive()) {
          double asidhfsadf = 0.3;
          HardwareRobot robot = new HardwareRobot(hardwareMap);
          DriveSubsystem drive = new DriveSubsystem (
            robot.leftBack,
            robot.leftFront,
            robot.rightBack,
            robot.rightFront
          );
          waitForStart();

      }
    }
}
