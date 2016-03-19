package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class DragonoidsTestController extends OpMode {
    @Override
    public void init() {
        DragonoidsGlobal.init(hardwareMap);
    }
    @Override
    public void loop() {
        DragonoidsGlobal.stopAll();
        this.outputTelemetry();
    }
    private void outputTelemetry() {
        telemetry.addData("1: ", gamepad1.toString());
        telemetry.addData("2: ", gamepad2.toString());
    }
    @Override
    public void stop() {
        // Stop all motors
        DragonoidsGlobal.stopAll();
        super.stop();
    }
}