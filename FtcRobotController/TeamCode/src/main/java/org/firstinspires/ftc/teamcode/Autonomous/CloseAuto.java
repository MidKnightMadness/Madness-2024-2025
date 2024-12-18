package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Close Auto")
public class CloseAuto extends AutoDrive{

    @Override
    public void park() {
        deadReckoningDrive.setMotorVelocitiesForTime(2.5, 0.5, 0, 0, 1000, telemetry);
    }

    @Override
    public void start() {
        park();
    }

    @Override
    public void loop(){
//        if(timer.updateTime() < timeRight){
//            mecanumDrive.velocityDrive(0.3, 0, 0, 1000);
//        }
//
//        if(timer.updateTime() > timeRight) {
//            mecanumDrive.setPowers(0, 0, 0, 0, 0);
//        }
    }



}

