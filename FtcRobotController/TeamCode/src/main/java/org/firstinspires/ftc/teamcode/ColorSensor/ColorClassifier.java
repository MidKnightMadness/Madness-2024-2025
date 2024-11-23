package org.firstinspires.ftc.teamcode.ColorSensor;

import org.firstinspires.ftc.teamcode.Helper.RGBColor;

public class ColorClassifier {

    public ColorClassifier() {
        // Color classification thresholds
    }

    public static SampleColors.Colors classify(RGBColor rgbColor) {
        double r = rgbColor.getR();
        double g = rgbColor.getG();
        double b = rgbColor.getB();

        // Check for very low RGB values to avoid classifying air
        if (r < 0.1 && g < 0.1 && b < 0.1) {
            return SampleColors.Colors.NONE; // Return NONE for air or very low light
        }

        // Classify colors based on RGB values
        if (r > 0.6 && g < 0.65 && b < 0.5) { // Condition for Red
            return SampleColors.Colors.RED;
        } else if (b > 0.5 && r < 0.45 && g < 0.5) { // Condition for Blue
            return SampleColors.Colors.BLUE;
        } else if (g > 0.55 && r > 0.3 && b < 0.5) { // Condition for Yellow
            return SampleColors.Colors.YELLOW;
        } else {
            return SampleColors.Colors.NONE; // Return NONE if no conditions are met
        }
    }
}

/*
package org.firstinspires.ftc.teamcode.ColorSensor;

import org.firstinspires.ftc.teamcode.Helper.RGBColor;

public class ColorClassifier {

    public ColorClassifier() {
        // Color classification thresholds
    }

    }
}
 */
