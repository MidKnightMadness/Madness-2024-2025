package org.firstinspires.ftc.teamcode.Helper;

public class ButtonToggle {
    boolean value;

    public ButtonToggle(){
        value = false;
    }

    public boolean update(boolean stateValue){
        if(stateValue && !value){
            value = true;
            return value;
        }
        value = false;

        return value;
    }


}
