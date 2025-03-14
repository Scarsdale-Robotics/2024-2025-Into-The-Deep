package org.firstinspires.ftc.teamcode.synchropather.systems.__util__;

import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleopWithNewSynchropather;

public class Command {
    public double speed;
    public String Action1;
    public String Action2;
    public String Action3;
    public String Action4;
    public String Action5;

    public void evalCommand () {

    }
    public void stopCommand() {

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

    public Command (double speed, String Action1, String Action2, String Action3, String Action4, String Action5
    , boolean isCommandFileRunning) {
        this.speed = speed;
        this.Action1 = Action1;
        this.Action2 = Action2;
        this.Action3 = Action3;
        this.Action4 = Action4;
        this.Action5 = Action5;
        this.isCommandFileRunning = isCommandFileRunning;
    }
}
