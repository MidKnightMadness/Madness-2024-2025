package org.firstinspires.ftc.teamcode.Benchmark;

import android.os.Environment;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Helper.Timer;

import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;

@TeleOp(name = "Update Rate Benchmark", group = "Benchmarks")

public class UpdateRateTestBenchmark extends OpMode {
    BufferedWriter bufferedWriter;
    FileWriter fileWriter;
    Timer timer;

    final int NUM_DATAPOINTS = 5000;
    int datapoints = 0;

    final static String OUTPUT_FILE = "benchmark.csv";


    @Override
    public void init() {

        timer = new Timer();

        String filePath = String.format("%s/FIRST/data/%s",
                Environment.getExternalStorageDirectory().getAbsolutePath(), OUTPUT_FILE);

        try {
            fileWriter = new FileWriter(filePath);
            bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.write("Delta time,Update Rate\n");

            telemetry.addLine("Successfully able to write to "+ OUTPUT_FILE);
        }
        catch (IOException e) {
            String errorMessage = e.getMessage();
            telemetry.addData("Error", errorMessage);
        }

    }

    @Override
    public void start() {
        telemetry.clear();
    }

    boolean stopped = false;
    @Override
    public void loop() {
        if (datapoints == NUM_DATAPOINTS) {
            if (!stopped) {
                stop();
                stopped = true;
            }
            else {
                telemetry.addLine("Stop robot");
            }
        }
        else {
            log();
            datapoints++;
        }
    }
    void log(AnalogInput in) {
        telemetry.addData("Voltage:", in.getVoltage());
        telemetry.addLine("-------------------------------");
    }


    @Override
    public void stop() {
        try {
            bufferedWriter.flush();
            bufferedWriter.close();
            fileWriter.close();
        }
        catch (IOException e) { }
    }

    void log() {
        timer.updateTime();
        writeData(timer.getDeltaTime(), 1.0 / timer.getDeltaTime());
    }

    void writeData(double deltaTime, double fps) {
        try {
            bufferedWriter.write(String.format("%f, %f\n",deltaTime, fps));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
}