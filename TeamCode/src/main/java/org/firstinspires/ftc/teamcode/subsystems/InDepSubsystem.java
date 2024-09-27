package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class InDepSubsystem extends SubsystemBase {

    private final HardwareRobot hardwareRobot;

    public InDepSubsystem(HardwareRobot hardwareRobot) {
        this.hardwareRobot = hardwareRobot;
    }

    //////////
    // CLAW //
    //////////
    public enum ClawPosition {
        DOWN(0),
        CENTER(0.5),
        UP(1);

        public final double SERVO_POSITION;

        private ClawPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setClawPosition(ClawPosition position) {
        setClawPosition(position.SERVO_POSITION);
    }
    public void setClawPosition(double position) {
        hardwareRobot.claw.setPosition(position);
    }

    ///////////
    // ELBOW //
    ///////////
    public enum ElbowPosition {
        CENTER(0.5),
        UPPER_CENTER(0.75),
        UP(1);

        public final double SERVO_POSITION;

        private ElbowPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setElbowPosition(ElbowPosition position) {
        setElbowPosition(position.SERVO_POSITION);
    }
    public void setElbowPosition(double position) {
        hardwareRobot.elbow.setPosition(position);
    }

}
