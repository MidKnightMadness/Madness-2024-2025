package org.firstinspires.ftc.teamcode.Autonomous;

import android.database.DefaultDatabaseErrorHandler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Drive.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@Autonomous
public class AutoDrive extends OpMode {
    Timer timer;
////    ErrorDrive errorDrive;
//    TwoWheelOdometry twoWheelOdometry;

    DeadReckoningDrive deadReckoningDrive;

    @Override
    public void init() {
        timer = new Timer();

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
//        errorDrive = new ErrorDrive(hardwareMap, telemetry);
//        twoWheelOdometry = new TwoWheelOdometry(hardwareMap, telemetry);

    }

    public void sleep(long time) throws InterruptedException {
        Thread.sleep(time);
    }

    public void park() {

    }

    @Override
    public void loop() {
    }
}
