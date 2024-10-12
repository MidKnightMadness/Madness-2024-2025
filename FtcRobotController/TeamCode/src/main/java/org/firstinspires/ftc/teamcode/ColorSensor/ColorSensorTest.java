package org.firstinspires.ftc.teamcode.ColorSensor;


//implement this class into Auto/TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;

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

        telemetry.addData("RGB Values", rgbColor.toString());
        telemetry.addData("Time", timer.updateTime());
        


    }

}
