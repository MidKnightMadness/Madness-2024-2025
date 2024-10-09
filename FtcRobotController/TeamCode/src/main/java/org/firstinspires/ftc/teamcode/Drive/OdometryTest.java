package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp
public class OdometryTest extends OpMode{

    MecanumDrive mecanumDrive;
    Timer timer;

    OdometryLinear linearOdo;
    OdometryArc arcOdo;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        timer = new Timer();


        OdometryArc arcOdo = new OdometryArc(hardwareMap, telemetry, new double[]{0,0});
        OdometryLinear linearOdo = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0});


    }

    double power = 0.5;
    @Override
    public void loop() {
        arcOdo.telemetry();

        telemetry.addLine("----------------------------------");

        linearOdo.telemtry();


        mecanumDrive.FieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);
    }
}
