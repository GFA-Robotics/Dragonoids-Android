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
        final double sliderForwardPower = 1.0;
        final double sliderBackwardPower = -0.5;
        if (gamepad1.right_bumper) {
            // Move the slider forward
            DragonoidsGlobal.rightSlider.setPower(sliderForwardPower);
            DragonoidsGlobal.leftSlider.setPower(sliderForwardPower);
        }
        else if (gamepad1.left_bumper) {
            // Reverse the slider
            DragonoidsGlobal.rightSlider.setPower(sliderBackwardPower);
            DragonoidsGlobal.leftSlider.setPower(sliderBackwardPower);
        }
        else {
            DragonoidsGlobal.rightSlider.setPower(0.0);
            DragonoidsGlobal.leftSlider.setPower(0.0);
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