package org.firstinspires.ftc.teamcode.synchropather.systems.__util__;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleopWithNewSynchropather;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;

public class Command {
    public double speed;
    public double nullVar;
    public double elbowUp = ElbowConstants.UP_POSITION;
    public double elbowDown = ElbowConstants.DOWN_POSITION;
    public double clawOpen = ClawConstants.OPEN_POSITION;
    public double clawClosed = ClawConstants.CLOSED_POSITION;
    public static String[] commandList;
    public RobotSystem robot;
    public Command (RobotSystem robot) {
        this.robot = robot;
    }
    public static void Add (String Action1, String Action2, String Action3, String Action4, String Action5) {
        commandList = new String[] {Action1, Action2, Action3, Action4, Action5};
    }
    public void evalCommand () {
        if (commandList != null) {
            startCommand();
        }
    }
    public void startCommand() {
        switch (commandList[1]) {
            case "EU":
                robot.inDep.setElbowPosition(elbowUp);
                break;
            case "ED":
                robot.inDep.setElbowPosition(elbowDown);
                break;
            case "null":
                nullVar = 0.3;
                break;
        }
        switch (commandList[2]) {
            case "EU":
                robot.inDep.setElbowPosition(elbowUp);
                break;
            case "ED":
                robot.inDep.setElbowPosition(elbowDown);
                break;
            case "null":
                nullVar = 0.3;
                break;
        }
        switch (commandList[3]) {
            case "EU":
                robot.inDep.setElbowPosition(elbowUp);
                break;
            case "ED":
                robot.inDep.setElbowPosition(elbowDown);
                break;
            case "null":
                nullVar = 0.3;
                break;
        }
        switch (commandList[4]) {
            case "EU":
                robot.inDep.setElbowPosition(elbowUp);
                break;
            case "ED":
                robot.inDep.setElbowPosition(elbowDown);
                break;
            case "null":
                nullVar = 0.3;
                break;
        }
    }
    public void initMode() {

    }

    public void switchMode() {

    }

    public void getElapsedTime() {

    }

    boolean isCommandFileRunning;

    public void resetTags() {
        //ill add this later, gotta study now :(
        //I'm starting to think this may be a really good idea.....
    }

}
