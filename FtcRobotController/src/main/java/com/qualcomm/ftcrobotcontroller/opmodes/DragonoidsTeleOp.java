package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class DragonoidsTeleOp extends OpMode {
    private boolean clampActivated = false;
    private boolean clampButtonPressed = false;

    @Override
    public void init() {
        DragonoidsGlobal.init(hardwareMap, true);
        // We don't need the LEDs on the color and light sensors to be on during tele-op
        DragonoidsGlobal.enableLEDs(true);
    }
    @Override
    public void start() {
        DragonoidsGlobal.engageDriveMotors();
    }
    @Override
    public void loop() {
        // Joystick values range from -1 to 1
        float forwardAmount = -gamepad1.left_stick_y;
        float turningAmount = -gamepad1.right_stick_x;

        // @galika1080 Starting to implement some wacky code for controlling the mecanum wheels
        // we're going to convert to polar, add pi/4 to theta, and convert back to cartesian.
        double r = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + math.pow(gamepad1.right_stick_y, 2));
        double t = Math.atan(y/x); // over to polar
        double newt = t + (Math.PI / 4); // adjust theta
        double processedX = Math.cos(newt)*r;
        double processedY = Math.sin(newt)*r; // back to cartesian
        // end @galika1080's wacky code

        forwardAmount = Range.clip(scaleInput(forwardAmount), -1, 1);
        turningAmount = Range.clip(scaleInput(turningAmount), -1, 1);

        double wheelPowerRF = Range.clip(-processedX, -1.0, 1.0);
        double wheelPowerLF = Range.clip(processedY, -1.0, 1.0);
        double wheelPowerRB = Range.clip(processedY, -1.0, 1.0);
        double wheelPowerLB = Range.clip(-processedX, -1.0, 1.0);

        /*
        double rightDrivePower = Range.clip(forwardAmount - turningAmount, -1.0, 1.0);
        double leftDrivePower = Range.clip(forwardAmount + turningAmount, -1.0, 1.0);
        DragonoidsGlobal.setDrivePower(rightDrivePower, leftDrivePower);
        */

        // Conveyor
        final double conveyorMaxPower = 0.70;
        final double conveyorMinPower = 0.45;
        if (gamepad2.right_bumper) {
            // Turn on the conveyor
            DragonoidsGlobal.conveyor.setPower(conveyorMaxPower);
        }
        else if (gamepad2.left_bumper) {
            // Reverse the conveyor
            DragonoidsGlobal.conveyor.setPower(-conveyorMinPower);
        }
        else {
            // Stop conveyor motor
            DragonoidsGlobal.conveyor.setPower(0.0);
        }

        // Climber release
        final double rightClimberOpen = 0.0;
        final double rightClimberClosed = 0.5;
        final double leftClimberOpen = 0.0;
        final double leftClimberClosed = 0.5;
        final double autonomousClimbersClosed = 1.0;
        final double autonomousClimbersOpen = 0.0;
        // Red B button for releasing climbers when in red alliance
        if (gamepad2.b) {
            DragonoidsGlobal.rightClimber.setPosition(rightClimberOpen);
        }
        else {
            DragonoidsGlobal.rightClimber.setPosition(rightClimberClosed);
        }
        // Blue X button for releasing climbers when in red alliance
        if (gamepad2.x) {
            DragonoidsGlobal.leftClimber.setPosition(leftClimberOpen);
        }
        else {
            DragonoidsGlobal.leftClimber.setPosition(leftClimberClosed);
        }
        // Y button for testing
        if (gamepad2.y) {
            DragonoidsGlobal.autonomousClimbers.setPosition(autonomousClimbersOpen);
        }
        else {
            DragonoidsGlobal.autonomousClimbers.setPosition(autonomousClimbersClosed);
        }

        // Sliders
        final double sliderForwardPower = 1.0;
        final double sliderBackwardPower = -0.8;
        if (gamepad2.right_trigger > 0.1) {
            // Move the sliders forward
            double sliderPower = gamepad2.right_trigger * sliderForwardPower;
            //DragonoidsGlobal.rightSlider.setPower(sliderPower);
            //DragonoidsGlobal.leftSlider.setPower(sliderPower);
        }
        else if (gamepad2.left_trigger > 0.1) {
            // Reverse the sliders
            double sliderPower = gamepad2.left_trigger * sliderBackwardPower;
            //DragonoidsGlobal.rightSlider.setPower(sliderPower);
            //DragonoidsGlobal.leftSlider.setPower(sliderPower);
        }
        else {
            //DragonoidsGlobal.rightSlider.setPower(0.0);
            //DragonoidsGlobal.leftSlider.setPower(0.0);
        }
        // Alternate control for sliders while the Logitech triggers aren't working properly
        if (gamepad2.dpad_up) {
            // Extend quickly
            DragonoidsGlobal.rightSlider.setPower(sliderForwardPower);
            DragonoidsGlobal.leftSlider.setPower(sliderForwardPower);
        }
        else if (gamepad2.dpad_right) {
            // Extend slowly
            DragonoidsGlobal.rightSlider.setPower(sliderForwardPower / 2.0);
            DragonoidsGlobal.leftSlider.setPower(sliderForwardPower / 2.0);
        }
        else if (gamepad2.dpad_down) {
            // Retract quickly
            DragonoidsGlobal.rightSlider.setPower(sliderBackwardPower);
            DragonoidsGlobal.leftSlider.setPower(sliderBackwardPower);
        }
        else if (gamepad2.dpad_left) {
            // Retract slowly
            DragonoidsGlobal.rightSlider.setPower(sliderBackwardPower / 2.0);
            DragonoidsGlobal.leftSlider.setPower(sliderBackwardPower / 2.0);
        }
        else {
            DragonoidsGlobal.rightSlider.setPower(0.0);
            DragonoidsGlobal.leftSlider.setPower(0.0);
        }

        // Dispenser
        final double dispenserMaxPower = 0.80;
        final double dispenserMidPower = 0.20;
        final double controlRange = 0.20;
        final double threshold = 0.10;
        // Negate values because pushing the stick forward yields a negative position
        float dispenserControl1 = gamepad2.left_stick_x;
        float dispenserControl2 = gamepad2.right_stick_x;
        if (Math.abs(dispenserControl1) > threshold) {
            // Control dispenser with greater power control
            DragonoidsGlobal.dispenser.setPower(Math.signum(dispenserControl1) * Range.scale(Math.abs(dispenserControl1), 0, 1, dispenserMaxPower - controlRange, dispenserMaxPower + controlRange));
        }
        else if (Math.abs(dispenserControl2) > threshold) {
            // Control dispenser with medium power control
            DragonoidsGlobal.dispenser.setPower(Math.signum(dispenserControl2) * Range.scale(Math.abs(dispenserControl2), 0, 1, dispenserMidPower - controlRange, dispenserMidPower + controlRange));
        }
        else {
            DragonoidsGlobal.dispenser.setPower(0.0);
        }

        // Clamps
        final double clampOpen = 0.0;
        final double clampClosed = 1.0;
        if (gamepad1.a && !clampButtonPressed) {
            clampButtonPressed = true;
            clampActivated = !clampActivated;
        }
        if (!gamepad1.a) {
            clampButtonPressed = false;
        }
        if (clampActivated) {
            DragonoidsGlobal.rightClamp.setPosition(clampClosed);
            DragonoidsGlobal.leftClamp.setPosition(clampClosed);
        }
        else {
            DragonoidsGlobal.rightClamp.setPosition(clampOpen);
            DragonoidsGlobal.leftClamp.setPosition(clampOpen);
        }

        this.outputTelemetry();
    }
    private void outputTelemetry() {
        //telemetry.addData("Right drive motor power", driveMotors.get("rightOneDrive").getPower());
        //telemetry.addData("Left drive motor power", driveMotors.get("leftOneDrive").getPower());
        //telemetry.addData("Conveyor motor power", DragonoidsGlobal.conveyor.getPower());
        telemetry.addData("Dispenser motor power", DragonoidsGlobal.dispenser.getPower());
        telemetry.addData("Right Two", DragonoidsGlobal.rightTwo.getCurrentPosition());
        telemetry.addData("Left  Two", DragonoidsGlobal.leftTwo.getCurrentPosition());
        telemetry.addData("Light sensor", DragonoidsGlobal.lightSensor.getLightDetectedRaw());
        //telemetry.addData("Servo Position", DragonoidsGlobal.gate.getPosition());
    }
    @Override
    public void stop() {
        // Stop all motors
        DragonoidsGlobal.stopAll();
        super.stop();
    }

    /*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
    private float scaleInputOriginal(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return (float)dScale;
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