package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Pose;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "PID Test")
@Disabled
public class PIDTest extends OpMode {

    PIDBasic pidBasic;
    ThreeWheelOdometry threeWheelOdometry;
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
        threeWheelOdometry = new ThreeWheelOdometry(hardwareMap, telemetry, startingPos);
        pidBasic = new PIDBasic(threeWheelOdometry, targetValues[0].getX(), targetValues[0].getY(), targetValues[0].getRotationRadians(), telemetry);
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
                pidBasic = new PIDBasic(threeWheelOdometry, targetValues[currentPoseTargetNumber].getX(),
                                                        targetValues[currentPoseTargetNumber].getY(),
                                                        targetValues[currentPoseTargetNumber].getRotationDegrees(),
                                                        telemetry);
            }
        }


    }
}
