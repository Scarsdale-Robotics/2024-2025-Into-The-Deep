package org.firstinspires.ftc.teamcode.opmodes.autons;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Objects;

@Autonomous(name="CVSampleAlign")
public class CVSampleAlign extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(2);

        telemetry.addData("state","prep");
        telemetry.update();

        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        ArrayList<Double> oldtxlist= new ArrayList<Double>();
        ArrayList<Double> oldtylist= new ArrayList<Double>();
        ArrayList<Double> runtimelist= new ArrayList<Double>();
        for(int i = 0; i<5;i++){
            oldtxlist.add(360.0);
            oldtylist.add(360.0);
            runtimelist.add(360.0);
        }

        double MAX_SPEED = 0.2;

        Date lt = new Date();
        double i=0;
        while (opModeIsActive()) {
            telemetry.addData("state","running");
            telemetry.addData("a", String.valueOf(i+=0.00001));
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                for (LLResultTypes.DetectorResult detection : detections) {
                    String className = detection.getClassName(); // What was detected
                    telemetry.addData("clr", className);
                    if (!Objects.equals(className, "yellow")) continue;
                    double tx = detection.getTargetXDegrees(); // Where it is (left-right)
                    double ty = detection.getTargetYDegrees(); // Where it is (up-down)

                    Date d2 = new Date();
                    double deltatime = (d2.getTime()-lt.getTime())/1000.0;
                    lt = d2;
                    runtimelist.remove(0);runtimelist.add(deltatime);

                    oldtxlist.remove(0);oldtxlist.add(deltatime);

                    if(oldtxlist.get(0)==360) {
                        drive.driveRobotCentric(Math.max(-MAX_SPEED, Math.min(tx * 0.05, MAX_SPEED)), -Math.max(-MAX_SPEED, Math.min(ty * 0.05, MAX_SPEED)), 0);
                    } else{
                        double derivativex = (-oldtxlist.get(4)+8*oldtxlist.get(3)-8*oldtxlist.get(1)+oldtxlist.get(0))/(12*(runtimelist.get(4)-runtimelist.get(0)));
                        double u_tx = Math.max(Math.min(tx*0.05+0.05*derivativex,MAX_SPEED),-MAX_SPEED);
                        double derivativey = (-oldtylist.get(4)+8*oldtylist.get(3)-8*oldtylist.get(1)+oldtylist.get(0))/(12*(runtimelist.get(4)-runtimelist.get(0)));
                        double u_ty = Math.max(Math.min(ty*0.05+0.5*derivativey,MAX_SPEED),-MAX_SPEED);
                        drive.driveRobotCentric(u_tx,-u_ty,0);
                    }
                    break;
                }
//                    telemetry.addData("Target X", tx);
//                    telemetry.addData("Target Y", ty);

                telemetry.update();
            } else {
                drive.driveRobotCentric(0, 0, 0);
                telemetry.addData("ERROR","NO RESULT");
                telemetry.update();
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
