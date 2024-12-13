package org.firstinspires.ftc.teamcode.Slides;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "Horizontal Slides")
public class HorizontalSlidesTest extends OpMode {
    HorizontalSlides horizontalSlides;
    ButtonToggle leftBumper;
    ButtonToggle rightBumper;
    ButtonToggle aBumper;

    @Override
    public void init() {
        horizontalSlides = new HorizontalSlides(hardwareMap);

        leftBumper = new ButtonToggle();
        rightBumper = new ButtonToggle();
        aBumper = new ButtonToggle();
    }

    boolean close = false;
    double maxTimeOpen = 0;
    double maxTimeClose = 0;
    Timer timer;
    double leftTriggerPos = 0;
    double previousTriggerVal = 0;

    boolean forward = true;
    @Override

    public void loop() {



        if(gamepad1.left_trigger != 0) {
            leftTriggerPos = gamepad1.left_trigger;
            previousTriggerVal = leftTriggerPos;
        }
        if(aBumper.update(gamepad1.a)){
            forward = !forward;
        }

        if(forward) {
            horizontalSlides.extend(leftTriggerPos);
            if(horizontalSlides.isRumble()){
                gamepad1.rumble(500);
            }
            leftTriggerPos = 0;
        }
        else{
            horizontalSlides.retract(leftTriggerPos);
            if(horizontalSlides.isRumble()){
                gamepad1.rumble(500);
            }
            leftTriggerPos = 0;
        }



        telemetry.addData("Forward", forward);
        telemetry.addData("Max Time Open", maxTimeOpen);
        telemetry.addData("Max Time Close", maxTimeClose);
        telemetry.addData("Left Trigger Pos", previousTriggerVal);

    }
}
