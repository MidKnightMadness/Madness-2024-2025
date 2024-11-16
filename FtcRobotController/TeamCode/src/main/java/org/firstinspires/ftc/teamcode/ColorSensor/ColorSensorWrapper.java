package org.firstinspires.ftc.teamcode.ColorSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode.Helper.RGBColor;

import java.util.ArrayList;

public class ColorSensorWrapper {
    ColorSensor colorSensor;
    int BUFFER_SIZE = 3;
    ArrayList<RGBColor> rgbBuffer = new ArrayList<RGBColor>();
    RGBColor value = new RGBColor();

    public ArrayList<RGBColor> getRgbBuffer() {
        return rgbBuffer;
    }


    public ColorSensorWrapper(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public ColorSensorWrapper(ColorSensor colorSensor, int bufferSize) {
        this.colorSensor = colorSensor;
        this.BUFFER_SIZE = bufferSize;
    }
    public void setWrapperSize(int bufferSize){
        this.BUFFER_SIZE = bufferSize;
    }

    public void clearBuffer() {
        this.rgbBuffer.clear();
    }

    public void update() {
        rgbBuffer.add(new RGBColor(colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()));
        //adding a new color the array list of rg colors
        //called many times in a frame
        if (this.BUFFER_SIZE == rgbBuffer.size()) {//if the number equals the size of the rg buffer in the array list
            value = RGBColor.average(rgbBuffer).normalizeRGB();//value which is a rgb color calls the average value of the array list
            //calculates the average of all the values in the arraylist rgbuffer and normalises them
            rgbBuffer.clear();//
        }
    }

    public RGBColor getValue() {
        return value;
    }
}
