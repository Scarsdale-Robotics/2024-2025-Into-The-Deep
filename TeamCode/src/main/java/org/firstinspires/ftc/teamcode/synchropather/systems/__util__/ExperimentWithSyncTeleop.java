package org.firstinspires.ftc.teamcode.synchropather.systems.__util__;

public abstract class ExperimentWithSyncTeleop {
    //viir's code for the teleop synchropather
    //this is the class containing the methods for new synchropather. ill change it from abstract to regular soon.
    //I'm supposed to document my process soooo lots of comments
    //I'll define later - methods for generic mode that recieve command files; time control for simplified actions.
    public abstract void startCommand ();

    public abstract void evalCommand();
    //for loop sifting through tags and actions of each command.
    public abstract void stopCommand();

    public abstract void initMode();

    public abstract void switchMode();

    public abstract void getElapsedTime();

    boolean isCommandFileRunning;
    //insert booleans here for all macros/command files desired to be run
}
