package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class DragonoidsTestSliders extends OpMode {
    @Override
    public void init() {
        DragonoidsGlobal.init(hardwareMap);
    }
    @Override
    public void loop() {
        final double sliderForwardPower = 0.75;
        final double sliderBackwardPower = -0.5;
        final int maxPosition = 5500;
        if (gamepad1.right_bumper) {
            // Move the slider forward
            //DragonoidsGlobal.rightSlider.setPower(sliderForwardPower);
            if (DragonoidsGlobal.leftSlider.getCurrentPosition() < maxPosition) {
                DragonoidsGlobal.leftSlider.setPower(sliderForwardPower);
            }
        }
        else if (gamepad1.left_bumper) {
            // Reverse the slider
            //DragonoidsGlobal.rightSlider.setPower(sliderBackwardPower);
            if (DragonoidsGlobal.leftSlider.getCurrentPosition() > 0) {
                DragonoidsGlobal.leftSlider.setPower(sliderBackwardPower);
            }
        }
        else {
            DragonoidsGlobal.rightSlider.setPower(0.0);
            DragonoidsGlobal.leftSlider.setPower(0.0);
        }
        if (gamepad1.a) {
            DragonoidsGlobal.rightSlider.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            DragonoidsGlobal.leftSlider.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
        else {
            DragonoidsGlobal.rightSlider.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            DragonoidsGlobal.leftSlider.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

        this.outputTelemetry();
    }
    private void outputTelemetry() {
        for (DcMotorController.RunMode value: DcMotorController.RunMode.values()) {
            if (value == DragonoidsGlobal.rightSlider.getMode()) {
                telemetry.addData("R Run Mode", value.name());
            }
            if (value == DragonoidsGlobal.leftSlider.getMode()) {
                telemetry.addData("L Run Mode", value.name());
            }
        }
        telemetry.addData("Right slider", DragonoidsGlobal.rightSlider.getCurrentPosition());
        telemetry.addData("Left  slider", DragonoidsGlobal.leftSlider.getCurrentPosition());
    }
    @Override
    public void stop() {
        // Stop all motors
        DragonoidsGlobal.stopAll();
        super.stop();
    }
}