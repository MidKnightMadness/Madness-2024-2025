package org.firstinspires.ftc.teamcode.Helper;

public class ButtonToggle {
    boolean value;

    public ButtonToggle(){
        value = false;
    }

    public boolean update(boolean stateValue){
        if(stateValue == true && value == false){
            value = true;
            return value;
        }
        value = false;
        return value;
    }


}
