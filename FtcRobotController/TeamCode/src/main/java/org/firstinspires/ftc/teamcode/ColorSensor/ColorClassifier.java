package org.firstinspires.ftc.teamcode.ColorSensor;

import org.firstinspires.ftc.teamcode.Helper.RGBColor;

public class ColorClassifier {

    public ColorClassifier(){
        //1. Yellow
        //    1. 0.57 - 0.60, 0.80, 0.2, 8200
        //2. Blue
        //    1. 0.25, 0.39, 0.90, 870 - 900
        //3. Red
        //    1. 0.86, 0.44, 0.23, 500
        //4. Regular
        //    1. 0.34, 0.66, 0.66, 70
    }

    public static SampleColors.Colors classify(RGBColor rgbColor){
        if(rgbColor.getR() > 0.6 && rgbColor.getG() < 0.65){ //4 inch range
            return(SampleColors.Colors.RED);
        }
        else if(rgbColor.getB()>0.5 && rgbColor.getR() < 0.45){ //2.5 inch range
            return(SampleColors.Colors.BLUE);
        }
        else if(rgbColor.getG() > 0.65){ //default is yellow
            return(SampleColors.Colors.YELLOW);
        }
        else{
            return(SampleColors.Colors.NONE);
        }
    }


}
