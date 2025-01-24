package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Servo3RArmConstants {

    public static double armBaseZero = 0.54;
    public static double armBaseNinety = 0.86;

    public static double armFirstJointZero = 0.32;
    public static double armFirstJointNinety = 0.64;

    public static double armSecondJointZero = 0.145;
    public static double armSecondJointNinety = 0.45;

    public static double armFirstSegmentLength = 9.21875;
    public static double armSecondSegmentLength = 12.5;
    public static final double maxRho = armSecondSegmentLength + armFirstSegmentLength;
    public static final double minRho = armSecondSegmentLength - armFirstSegmentLength;

}
