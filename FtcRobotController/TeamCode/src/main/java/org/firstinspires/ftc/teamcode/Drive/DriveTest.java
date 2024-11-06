package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp
public class DriveTest extends OpMode {
    PIDBasic PIDDrive;
    OdometryLinear odometry;
    MecanumDrive mecanumDrive;
    public double[] targetStates = {40, 60, 20};
    Timer timer;

    @Override
    public void init() {
        timer = new Timer();
        odometry = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0, Math.PI/2});
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        telemetry.update();

    }

    double[] pidOutputs = new double[3];
    double drivePower = 0.5;
    @Override
    public void loop() {
        odometry.update();

//
//        //POINT to POINT PID
//        PIDDrive = new PIDBasic(odometry, targetStates[0], targetStates[1], targetStates[2], telemetry);
//        pidOutputs = PIDDrive.updatePID();
//        mecanumDrive.FieldOrientedDrive(pidOutputs[0], pidOutputs[1], pidOutputs[2], drivePower);
//        telemetry.addLine("Point to Point PID");

        //Driver Field Oriented
        if(gamepad1.left_bumper){
            odometry.update();
           // mecanumDrive.FieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
            telemetry.addLine("Driver Field Oriented");
        }
        //Robot oriented
        else{
            odometry.update();
            mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
            telemetry.addLine("Robot Oriented");
        }

        telemetry.addData("Robot X", odometry.getXCoordinate());
        telemetry.addData("Robot Y", odometry.getYCoordinate());
        telemetry.addData("Robot Theta", odometry.getRotationDegrees());

        telemetry.addData("Robot X Velocity", odometry.getVelocityX());
        telemetry.addData("Robot Y Velocity", odometry.getVelocityY());

        telemetry.addData("Time", timer.updateTime());

    }
}
