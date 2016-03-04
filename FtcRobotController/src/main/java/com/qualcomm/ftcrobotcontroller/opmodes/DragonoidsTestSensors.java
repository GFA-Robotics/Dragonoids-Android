package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class DragonoidsTestSensors extends OpMode {
    private ColorSensor colorSensor;
    private OpticalDistanceSensor opticalDistanceSensor;
    private LightSensor lightSensor;
    @Override
    public void init() {
        DragonoidsGlobal.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("color");
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("distance");
        lightSensor = hardwareMap.lightSensor.get("light");
        lightSensor.enableLed(true);
        // Change for testing
        colorSensor.enableLed(true);
    }
    @Override
    public void loop() {
        DragonoidsGlobal.stopAll();
        this.outputTelemetry();
    }
    private void outputTelemetry() {
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("ODS", opticalDistanceSensor.getLightDetected());
        telemetry.addData("Line light", lightSensor.getLightDetected());
        telemetry.addData("Line light raw", lightSensor.getLightDetectedRaw());
    }
    @Override
    public void stop() {
        // Stop all motors
        DragonoidsGlobal.stopAll();
        super.stop();
    }
}