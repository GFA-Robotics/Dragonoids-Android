package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class DragonoidsTeleOpSimple extends OpMode {
    private final double maxDrivePower = 0.6;
    private final double maxDispenserPower = 0.50;
    private final double maxConveyorPower = 0.70;

    private final double minDistanceThreshold = 0.08;

    @Override
    public void init() {
        DragonoidsGlobal.init(hardwareMap);
    }
    @Override
    public void loop() {
        this.outputTelemetry();

        if (DragonoidsGlobal.opticalDistanceSensor.getLightDetected() > minDistanceThreshold) {
            // Enforce a minimum distance to avoid collisions
            DragonoidsGlobal.stopAll();
            return;
        }

        // Joystick values range from -1 to 1
        float forwardAmount = -gamepad1.left_stick_y;
        float turningAmount = -gamepad1.right_stick_x;

        forwardAmount = Range.clip(scaleInput(forwardAmount), -1, 1);
        turningAmount = Range.clip(scaleInput(turningAmount), -1, 1);

        //TEMPORARY HACK OF A FIX OH GOD NO PLEASE FIX THIS
        double rightDrivePower = Range.clip(forwardAmount - turningAmount, -maxDrivePower, maxDrivePower);
        double leftDrivePower = Range.clip(forwardAmount + turningAmount, -maxDrivePower, maxDrivePower);
        double rearLeftDrivePower = Range.clip(-)
        DragonoidsGlobal.setDrivePower(rightDrivePower, leftDrivePower);

        // Conveyor
        if (gamepad1.right_trigger > 0.8) {
            // Turn on the conveyor
            DragonoidsGlobal.conveyor.setPower(maxConveyorPower * (3.0 / 3.0));
        }
        else if (gamepad1.right_trigger > 0.4) {
            DragonoidsGlobal.conveyor.setPower(maxConveyorPower * (2.0 / 3.0));
        }
        else if (gamepad1.right_trigger > 0.1) {
            DragonoidsGlobal.conveyor.setPower(maxConveyorPower * (1.0 / 3.0));
        }
        else if (gamepad1.left_trigger > 0.2) {
            // Reverse the conveyor
            DragonoidsGlobal.conveyor.setPower(-1 * maxConveyorPower * (2.0 / 3.0));
        }
        else {
            // Stop conveyor motor
            DragonoidsGlobal.conveyor.setPower(0.0);
        }

        // Dispenser
        if (gamepad1.right_bumper) {
            DragonoidsGlobal.dispenser.setPower(maxDispenserPower);
        }
        else if (gamepad1.left_bumper) {
            DragonoidsGlobal.dispenser.setPower(-maxDispenserPower);
        }
        else {
            DragonoidsGlobal.dispenser.setPower(0.0);
        }
    }
    private void outputTelemetry() {
        telemetry.addData("ODS reading", DragonoidsGlobal.opticalDistanceSensor.getLightDetected());
    }
    @Override
    public void stop() {
        // Stop all motors
        DragonoidsGlobal.stopAll();
        super.stop();
    }

    private float scaleInput (double value) {
        // Return the value (from -1 to 1) squared to scale it quadratically
        float magnitude = (float) Math.pow(value, 2);
        if (value < 0) {
            return -1 * magnitude;
        }
        else {
            return magnitude;
        }
    }
}