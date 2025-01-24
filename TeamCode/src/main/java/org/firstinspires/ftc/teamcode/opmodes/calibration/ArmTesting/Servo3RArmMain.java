package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

public class Servo3RArmMain {
    public static void main(String[] args) {
        Servo3RArmIK servo3RArm = new Servo3RArmIK();
        servo3RArm.getServoArmPositions(new double[]{6.76233287332, 0, -2.45764534466});
        double[] servoAnglesJacobian = servo3RArm.getServoAnglesJacobian(
                new double[]{8.71373441482, 0, -1.50702216417}
        );


        System.out.println(servoAnglesJacobian[0]); //  0.0
        System.out.println(servoAnglesJacobian[1]); // -0.249636452971
        System.out.println(servoAnglesJacobian[2]); //  0.932901030556
    }
}
