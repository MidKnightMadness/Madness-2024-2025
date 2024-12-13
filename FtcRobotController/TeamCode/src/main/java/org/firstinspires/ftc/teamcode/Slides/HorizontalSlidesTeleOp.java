package org.firstinspires.ftc.teamcode.Slides;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;
@TeleOp(name = "Horizontal Slides TeleOp")
public class HorizontalSlidesTeleOp extends OpMode {
    ButtonToggle leftBumper;
    ButtonToggle a1Bumper;
    Servo horizontalRight;
    Servo horizontalLeft;
    Timer timer;
    Timer overallTimer;
    double maxEntensionTime = 2.5;
    double maxRetractionTime = 2.5;
    boolean rumble = false;
    CRServo crHorizontalLeft;
    CRServo crHorizontalRight;

    @Override
    public void init() {
        crHorizontalLeft = hardwareMap.get(CRServo.class, "Horizontal Slides Left");
        crHorizontalRight = hardwareMap.get(CRServo.class, "Horizontal Slides Right");

        //idk why direction doesnt work too well
        crHorizontalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        crHorizontalRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBumper = new ButtonToggle();

    }

    double maxTimeOpen = 0;
    double maxTimeClose = 0;
    double leftTriggerPos = 0;
    double previousTriggerVal = 0;

    boolean forward = true;
    double overallTimeExtend;
    double overallTimeRetract;
    @Override

    public void loop() {
        double upperValue = (gamepad1.left_stick_x + 3.0) / 4.0; //0.5 - 1
        double bottomValue = (gamepad1.left_stick_x + 1.0) / 4.0;//0 - 0.5

        if(forward == true){
            crHorizontalLeft.setPower(upperValue);
            crHorizontalRight.setPower(bottomValue);
        }
        else{
            crHorizontalLeft.setPower(bottomValue);
            crHorizontalRight.setPower(upperValue);
        }


        if(a1Bumper.update(gamepad1.a)){
            forward = !forward;
        }
//
//        if(forward) {
//            this.extend(leftTriggerPos);
//            leftTriggerPos = 0;
//        }
//        else{
//            this.retract(leftTriggerPos);
//            leftTriggerPos = 0;
//        }
//
//        if(x1Bumper.update(gamepad1.x)){
//            overallTimer = new Timer();
//            this.extend(1);
//            overallTimeExtend = overallTimer.updateTime();
//        }
//
//        if(y1Bumper.update(gamepad1.y)){
//            overallTimer = new Timer();
//            this.retract(1);
//            overallTimeRetract = overallTimer.updateTime();
//        }



        telemetry.addLine("-----------------");
        telemetry.addData("Direction", forward == true ? "Extend" : "Retract");
        telemetry.addData("Left Power", forward == true ? upperValue : bottomValue);
        telemetry.addData("Right Power", forward == true ? bottomValue : upperValue);

        telemetry.addLine("-----------------");
        telemetry.addData("Max Time Open", maxTimeOpen);
        telemetry.addData("Max Time Close", maxTimeClose);
        telemetry.addData("Left Trigger Pos", previousTriggerVal);
        telemetry.addData("Rumble", rumble);
        telemetry.addData("Rumble Time", rumbleTime);
        telemetry.addLine("-----------------");

        telemetry.addData("Overall Time Extend", overallTimeExtend);
        telemetry.addData("Overall Time Retract", overallTimeRetract);


    }





    public void extendByTime(double percent) {
        timer = new Timer();
        double totalTime = percent * maxEntensionTime;
        double currentTime = timer.updateTime();

        while (currentTime < totalTime) {
            currentTime = timer.updateTime();
            horizontalLeft.setDirection(Servo.Direction.FORWARD);
            horizontalLeft.setPosition(Servo.MAX_POSITION);
            if(totalTime - currentTime <= 0.5 && rumble == false){
                rumbleTime = (int) (1000 *  (totalTime - currentTime));
                gamepad1.rumble(rumbleTime);
            }
        }
        rumble = false;
        horizontalLeft.setPosition(0.5);

//        horizontalRight.setPosition(percent * servoRightBounds[1]);
//        horizontalLeft.setPosition(horizontalLeft.getPosition() + 0.1*x);
//        horizontalRight.setPosition(horizontalRight.getPosition() + 0.1*x);
    }


    int rumbleTime = 0;
    public void retractByTime(double percent){
        timer = new Timer();
        double totalTime = percent * maxRetractionTime;
        double currentTime = timer.updateTime();

        while (currentTime < totalTime) {
            currentTime = timer.updateTime();
            horizontalLeft.setDirection(Servo.Direction.FORWARD);
            horizontalLeft.setPosition(Servo.MIN_POSITION);
            if(totalTime - currentTime <= 0.5 && rumble == false){
                rumble = true;
                rumbleTime = (int) (1000 *  (totalTime - currentTime));
                gamepad1.rumble(rumbleTime);
            }
        }
        rumble = false;
        horizontalLeft.setPosition(0.5);
        //  horizontalLeft.setPosition(servoRightBounds[0]);
    }




    public void stop(){
        horizontalLeft.setPosition(0.5);
        horizontalRight.setPosition(0.5);
    }
    public double getLeftPosition(){
        return horizontalLeft.getPosition();
    }

}
