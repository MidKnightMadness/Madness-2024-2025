package org.firstinspires.ftc.teamcode.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "Distance Sensor Test")
@Disabled
public class DistanceSensorTest extends OpMode {

    DistanceSensor distanceSensor;
    DistanceSensorBuffer distanceSensorBuffer;
    Timer timer;
    MecanumDrive mecanumDrive;
    double specimenDistance;
    double power = 0.8;
    ButtonToggle leftBumper;
    ButtonToggle dPadRightToggle;
    double buttonToggle = 5;
    @Override
    public void init() {
        dPadRightToggle = new ButtonToggle();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        distanceSensorBuffer = new DistanceSensorBuffer(distanceSensor, 5);
        timer = new Timer();
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        leftBumper = new ButtonToggle();
    }

    @Override
    public void loop() {
        if(dPadRightToggle.update(gamepad1.right_bumper)){
            distanceSensorBuffer.setBufferSize(buttonToggle + 1);
        }
        if(leftBumper.update(gamepad1.left_bumper)) {
            if(power == 0.4) {
                power = 0.8;
            }
            if(power == 0.8){
                power = 0.4;
            }
            gamepad1.rumble(100);
        }

        mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);

        telemetry.addData("Time", timer.updateTime());

        telemetry.addLine("Test");
        telemetry.addData("Distance Sensor Distance(IN)", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance Sensor Distance(CM)", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Buffer Size", distanceSensorBuffer.getBufferSize());

        distanceSensorBuffer.update();
        telemetry.addData("Normalized Distance Sensor(IN)",  distanceSensorBuffer.getNormalizedDistance());
        telemetry.addData("Normalized Distance Sensor Distance(CM)", distanceSensorBuffer.getNormalizedDistance() * 2.54);

        telemetry.update();
        mecanumDrive.updateTelemetry();
    }
}
