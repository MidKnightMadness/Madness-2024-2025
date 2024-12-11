package org.firstinspires.ftc.teamcode.Slides;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;

@TeleOp(name = "Horizontal Slides")
public class HorizontalSlidesTest extends OpMode {
    HorizontalSlides horizontalSlides;
    ButtonToggle buttonToggleA;
    ButtonToggle buttonToggleX;
    @Override
    public void init() {
        horizontalSlides = new HorizontalSlides(hardwareMap);
        buttonToggleA = new ButtonToggle();
        buttonToggleX = new ButtonToggle();
    }

    boolean close = false;
    @Override

    public void loop() {
        if(close == true) {
            if(buttonToggleX.update(gamepad1.x)){
                horizontalSlides.extend();
                telemetry.addLine("Horizontal Slides Extended");
                close = false;
            }
        }

        if(close == false) {
            if (buttonToggleA.update(gamepad1.a)) {
                horizontalSlides.close();
                telemetry.addLine("Horizontal Slides Closed");
                close = true;
            }
        }

    }
}
