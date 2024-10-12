package org.firstinspires.ftc.teamcode.Helper;


import java.util.ArrayList;

public class RGBColor {
    public float r;
    public float g;
    public float b;
    public float a;

    public float getR() {
        return r;
    }

    public void setR(float r) {
        this.r = r;
    }

    public float getG() {
        return g;
    }

    public void setG(float g) {
        this.g = g;
    }

    public float getB() {
        return b;
    }

    public void setB(float b) {
        this.b = b;
    }

    public float getA() {
        return a;
    }

    public void setA(float a) {
        this.a = a;
    }



    public RGBColor() {
        this.r = 0;
        this.g  = 0;
        this.b = 0;
        this.a = 0;
    }

    public RGBColor(float r, float g, float b, float a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    public String toString() {
        return String.format("(%s, %s, %s, %s)", this.r, this.g, this.b, this.a);
    }

    public static RGBColor average(ArrayList<RGBColor> colors) {
        float rSum = 0;
        float gSum = 0;
        float bSum = 0;
        float aSum = 0;

        float length = colors.size();

        for (int i = 0; i < colors.size(); i++) {
            rSum += colors.get(i).r;
            gSum += colors.get(i).g;
            bSum += colors.get(i).b;
            aSum += colors.get(i).a;
        }

        return new RGBColor(rSum / length, gSum / length, bSum / length, aSum / length);
    }

    public RGBColor normalizeRGB() {
        float magnitude = (float) Math.sqrt(this.r * this.r + this.g * this.g + this.b * this.b);
        float adjustedMagnitude = magnitude;

        this.r /= adjustedMagnitude;
        this.g /= adjustedMagnitude;
        this.b /= adjustedMagnitude;

        return this;
    }

    public RGBColor normalizeRGB(float targetMagnitude) {
        float magnitude = (float) Math.sqrt(this.r * this.r + this.g * this.g + this.b * this.b);
        float adjustedMagnitude = magnitude / targetMagnitude;

        this.r /= adjustedMagnitude;
        this.g /= adjustedMagnitude;
        this.b /= adjustedMagnitude;

        return this;
    }
}
