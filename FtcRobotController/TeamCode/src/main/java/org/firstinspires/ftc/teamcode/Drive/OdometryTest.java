package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Drive.OdometryArc;
import org.firstinspires.ftc.teamcode.Drive.OdometryLinear;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp(name = "Odometry Test")
public class OdometryTest extends OpMode{

    MecanumDrive mecanumDrive;
    OdometryLinear linearOdo;
    ButtonToggle left1Bumper, right1Bumper;



    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        left1Bumper = new ButtonToggle();
        right1Bumper = new ButtonToggle();

        linearOdo = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0, 0});
    }

    boolean isUsingFieldOriented;
    double drivePower = 0.8;
    @Override
    public void loop() {

        if(left1Bumper.update(gamepad1.left_bumper)){
            isUsingFieldOriented = !isUsingFieldOriented;
            gamepad1.rumble(100);
        }


        if(right1Bumper.update(gamepad1.right_bumper)){
            if(drivePower == 0.8){
                drivePower = 0.5;
                gamepad1.rumble(300);
            }
            else if(drivePower == 0.5){
                drivePower = 0.3;
                gamepad1.rumble(400);
            }
            else{
                drivePower = 0.8;
                gamepad1.rumble(500);
            }
        }

        linearOdo.update();


        telemetry.addLine("----------------------------------");

        linearOdo.telemtry();

        if(isUsingFieldOriented){
            mecanumDrive.velocityDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 1000);
            telemetry.addLine("Velocity Drive");
        }
        else {
            mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
            telemetry.addLine("Normal Drive");
        }

    }
}
