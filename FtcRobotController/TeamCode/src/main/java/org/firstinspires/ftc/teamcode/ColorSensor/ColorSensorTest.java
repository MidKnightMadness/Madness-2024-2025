package org.firstinspires.ftc.teamcode.ColorSensor;


//implement this class into Auto/TeleOp

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "Color Sensor")
@Disabled
public class ColorSensorTest extends OpMode {
    ColorSensor colorSensor;
    ColorSensorWrapper colorSensorWrapper;
    Timer timer;
    public int bufferSize = 3;
    @Override
    public void init() {
        timer = new Timer();

        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, bufferSize);
    }

    @Override
    public void loop() {
        colorSensorWrapper.update();
        RGBColor rgbColor = colorSensorWrapper.getValue();

        SampleColors.Colors detected = ColorClassifier.classify(rgbColor);

        telemetry.addData("Detected Color", detected);
        telemetry.addData("RGB Values", rgbColor.toString());
        telemetry.addData("Time", timer.updateTime());
        
        telemetry.update();

    }

}
