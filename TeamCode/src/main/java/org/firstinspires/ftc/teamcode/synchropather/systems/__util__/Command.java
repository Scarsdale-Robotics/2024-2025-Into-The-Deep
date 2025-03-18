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
    public double elbowUp = ElbowConstants.UP_POSITION;
    public double elbowDown = ElbowConstants.DOWN_POSITION;
    public double clawOpen = ClawConstants.OPEN_POSITION;
    public double clawClosed = ClawConstants.CLOSED_POSITION;
    boolean isClawOccupied = false;
    boolean isElbowOccupied = false;
    public static boolean isCommandFileRunning = false;
    public static String[] commandList;
    public RobotSystem robot;
    public Command (RobotSystem robot) {
        this.robot = robot;
    }
    public static void Add (String Action1, String Action2, String Action3, String Action4, String Action5) {
        commandList = new String[] {Action1, Action2, Action3, Action4, Action5};
    }
    public void evalCommand () {
        if (commandList != null && !isCommandFileRunning) {
            isCommandFileRunning = true;
            startCommand();
        }
    }
    public static void resetCommand() {
        //add conditions for tags and when to stop/start next command
        isCommandFileRunning = false;
        commandList = null;
    }
    public void startCommand() {
        if (isClawOccupied && isElbowOccupied) {
            Command.resetCommand();
            isClawOccupied = false;
            isElbowOccupied = false;
        }
        //sample evaluation
        if (isCommandFileRunning) {
            switch (commandList[1]) {
                case "EU":
                    robot.inDep.setElbowPosition(elbowUp);
                    isElbowOccupied = true;
                    break;
                case "CC":
                    robot.inDep.setClawPosition(clawClosed);
                    isClawOccupied = true;
                    break;
                case "ED":
                    robot.inDep.setElbowPosition(elbowDown);
                    isElbowOccupied = true;
                    break;
                case "CO":
                    robot.inDep.setClawPosition(clawOpen);
                    isClawOccupied = true;
                    break;
                case "":
                    break;
            }
            switch (commandList[2]) {
                case "EU":
                    robot.inDep.setElbowPosition(elbowUp);
                    isElbowOccupied = true;
                    break;
                case "CC":
                    robot.inDep.setClawPosition(clawClosed);
                    isClawOccupied = true;
                    break;
                case "ED":
                    robot.inDep.setElbowPosition(elbowDown);
                    isElbowOccupied = true;
                    break;
                case "CO":
                    robot.inDep.setClawPosition(clawOpen);
                    isClawOccupied = true;
                    break;
                case "":
                    break;
            }
            switch (commandList[3]) {
                case "EU":
                    robot.inDep.setElbowPosition(elbowUp);
                    isElbowOccupied = true;
                    break;
                case "CC":
                    robot.inDep.setClawPosition(clawClosed);
                    isClawOccupied = true;
                    break;
                case "ED":
                    robot.inDep.setElbowPosition(elbowDown);
                    isElbowOccupied = true;
                    break;
                case "CO":
                    robot.inDep.setClawPosition(clawOpen);
                    isClawOccupied = true;
                    break;
                case "":
                    break;
            }
            switch (commandList[4]) {
                case "EU":
                    robot.inDep.setElbowPosition(elbowUp);
                    isElbowOccupied = true;
                    break;
                case "CC":
                    robot.inDep.setClawPosition(clawClosed);
                    isClawOccupied = true;
                    break;
                case "ED":
                    robot.inDep.setElbowPosition(elbowDown);
                    isElbowOccupied = true;
                    break;
                case "CO":
                    robot.inDep.setClawPosition(clawOpen);
                    isClawOccupied = true;
                    break;
                case "":
                    break;
            }
        }
        resetCommand();
    }
    //mode method for initializing certain servos. (not really necessary)
    public void initMode() {

    }
    public void switchMode() {

    }
    //method for coordinating actions in commandList
    public void getElapsedTime() {

    }


//buttons and servos are tagged as used or unused.
    public void resetTags() {
        //ill add this later, gotta study now :(
        //I'm starting to think this may be a really good idea.....
    }

}
