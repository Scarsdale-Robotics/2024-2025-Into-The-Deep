package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="ari LimelightSh_t")
public class LimelightSh_t extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(2);

        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );
        LocalizationSubsystem localization = new LocalizationSubsystem(
                new Pose2d(0, 0, new Rotation2d(0)),
                robot.leftOdometer,
                robot.rightOdometer,
                robot.centerOdometer,
                telemetry
        );
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        List<Double> oldtxlist= Arrays.asList(360.0,360.0,360.0,360.0);

        ElapsedTime runtime = new ElapsedTime(0);

        List<Double> runtimelist= Arrays.asList(360.0,360.0,360.0,360.0);
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa();

                    double deltatime = runtime.seconds();
                    runtimelist.remove(0);runtimelist.add(deltatime);

                    oldtxlist.remove(0);oldtxlist.add(deltatime);

                    if(oldtxlist.get(0)==360) {
                        drive.driveRobotCentric(tx * 0.01, 0, 0);
                    }
                    else{
                        double derivativex = (-oldtxlist.get(4)+8*oldtxlist.get(3)-8*oldtxlist.get(1)+oldtxlist.get(0))/(12*(runtimelist.get(4)-runtimelist.get(0)));
                    }
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);

                    telemetry.update();
                }
            }

        }
    }
    public void driveuntilcentered(double Totaldeltay, double Totaldeltax, DriveSubsystem drive){
        double kp = 0.001;
        double kd = 0.0001;

        double startpos = drive.getRightBackPosition();
        double frontwheelstartpos = drive.getRightFrontPosition();
        ArrayList<Double> errorlist = new ArrayList<Double>();
        ArrayList<Double> errorlistx = new ArrayList<Double>();
        ArrayList<Double> runtimelist = new ArrayList<Double>();
        for(int i =0;i<5;i++){
            errorlist.add(Totaldeltay);
            errorlistx.add(Totaldeltay);
            runtimelist.add(0.0);
        }
        ElapsedTime runtime = new ElapsedTime(0);
        double error = 11;
        while (opModeIsActive() && Math.abs(error) > 10){
            // calc error
            double encoderposition = drive.getRightBackPosition()-startpos;
            double frontwheel = drive.getRightFrontPosition()-frontwheelstartpos;
            double xchange = encoderposition-frontwheel;
            error = Totaldeltay-encoderposition;
            double errorx = Totaldeltax - xchange;
            errorlist.remove(0);errorlist.add(error);
            errorlistx.remove(0);errorlistx.add(errorx);
            double deltatime = runtime.seconds();
            runtimelist.remove(0);runtimelist.add(deltatime);

            // do 5 point derivative
            double derivative = (-errorlist.get(4)+8*errorlist.get(3)-8*errorlist.get(1)+errorlist.get(0))/(12*(runtimelist.get(4)-runtimelist.get(0)));
            double derivativex = (-errorlistx.get(4)+8*errorlistx.get(3)-8*errorlistx.get(1)+errorlistx.get(0))/(12*(runtimelist.get(4)-runtimelist.get(0)));

            // movement
            double u_t = Math.max(Math.min(kp * error + kd * derivative,0.6),-0.6);
            double u_tx = Math.max(Math.min(kp * errorx + kd * derivativex,0.6),-0.6);
            drive.driveRobotCentric(0,u_t, 0);

            //show info
            telemetry.addData("encoderposition",encoderposition);
            //telemetry.addData("xpos:", xchange);
            //telemetry.addData("P",kp * error);
            //telemetry.addData("D",kd * derivative);
            telemetry.addData("U_T",u_t);
            telemetry.addData("U_TX",u_tx);
            telemetry.update();
        }

    }
}
