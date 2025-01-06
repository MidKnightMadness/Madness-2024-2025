package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Auto")
public class FarAuto extends AutoDrive{

    @Override
    public void park() {

        try {
            Thread.sleep(8000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        deadReckoningDrive.setMotorVelocitiesForTime(1.5, -0.5, 0, 0, 1000, telemetry);
        deadReckoningDrive.setMotorVelocitiesForTime(1.5, -0.5, 0, 0, 1000, telemetry);

        deadReckoningDrive.setMotorVelocitiesForTime(1.5, 0, 0.5, 0, 1000, telemetry);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // RPM of wheels: vel * 60 / 2pi / 16

        deadReckoningDrive.setMotorVelocitiesForTime(5, 0.5, 0, 0, 1000, telemetry);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        deadReckoningDrive.setMotorVelocitiesForTime(2, 0, -0.5, 0, 1000, telemetry);
    }

    @Override
    public void start() {
        park();
    }
}
