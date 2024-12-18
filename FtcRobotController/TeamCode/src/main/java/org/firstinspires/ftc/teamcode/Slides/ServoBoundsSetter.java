package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoBoundsSetter extends OpMode {

    Servo leftWristServo;
    Servo rightWristServo;
    Servo claw;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "clawServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
//        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
    }

    String activeServo = "Left wrist";
    double servoPosition = 0.5;

    @Override
    public void loop() {
        if (gamepad1.a) {
            activeServo = "Left wrist";
        }
        if (gamepad1.x) {
            activeServo = "Right wrist";
        }
        if (gamepad1.y) {
            activeServo = "Claw";
        }

        if (gamepad1.dpad_up) {
            servoPosition += 0.001;
        }

        if (gamepad1.dpad_down) {
            servoPosition -= 0.005;
        }

        if (activeServo.equals("Left wrist")) {
            leftWristServo.setPosition((servoPosition));
        }
//        else if (activeServo.equals("Right wrist")) {
//            rightWristServo.setPosition(servoPosition);
//        }
        else {
            claw.setPosition(servoPosition);
        }

        telemetry.addData("Active servo", activeServo);
        telemetry.addData("Servo position", servoPosition);
    }
}
