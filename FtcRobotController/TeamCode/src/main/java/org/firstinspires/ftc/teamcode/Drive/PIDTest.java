package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Pose;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "PID Test")
public class PIDTest extends OpMode {

    PIDBasic pidBasic;
    OdometryLinear odometryLinear;
    Timer timer;
    double[] startingPos = new double[]{7.5, 7.5};
    ButtonToggle leftBumperToggle;

    Pose[] targetValues = new Pose[]{
            new Pose(10, 20, Math.PI / 2),
            new Pose(40, 50, Math.PI / 2),
            new Pose(120, 120, 0)
    };


    @Override
    public void init() {
        odometryLinear = new OdometryLinear(hardwareMap, telemetry, startingPos);
        pidBasic = new PIDBasic(odometryLinear, targetValues[0].getX(), targetValues[0].getY(), targetValues[0].getRotationRadians(), telemetry);
        leftBumperToggle = new ButtonToggle();
        timer = new Timer();
    }

    int currentPoseTargetNumber = 0;
    @Override
    public void loop() {
        telemetry.addLine("Running");
        pidBasic.updatePID();

        //switch to next target
        if(pidBasic.distancetoTarget < 3 && leftBumperToggle.update(gamepad1.left_bumper)){
            if((currentPoseTargetNumber + 1) < targetValues.length){
                currentPoseTargetNumber += 1;
                pidBasic = new PIDBasic(odometryLinear, targetValues[currentPoseTargetNumber].getX(),
                                                        targetValues[currentPoseTargetNumber].getY(),
                                                        targetValues[currentPoseTargetNumber].getRotationDegrees(),
                                                        telemetry);
            }
        }


    }
}
