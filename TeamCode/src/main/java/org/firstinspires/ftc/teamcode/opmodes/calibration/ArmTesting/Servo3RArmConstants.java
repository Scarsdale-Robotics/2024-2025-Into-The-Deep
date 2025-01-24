package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Servo3RArmConstants {

    public static double armBaseZero = 0.53;
    public static double armBaseNinety = 0.85;

    public static double armFirstJointZero = 0.34;
    public static double armFirstJointNinety = 0.66;

    public static double armSecondJointZero = 0.145;
    public static double armSecondJointNinety = 0.45;

    public static double armFirstSegmentLength = 10;
    public static double armSecondSegmentLength = 10;
    public static final double maxRho = armSecondSegmentLength + armFirstSegmentLength;
    public static final double minRho = armSecondSegmentLength - armFirstSegmentLength;

}
