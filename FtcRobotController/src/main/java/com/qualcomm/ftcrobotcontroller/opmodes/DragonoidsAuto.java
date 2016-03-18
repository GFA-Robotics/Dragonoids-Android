package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DragonoidsAuto extends LinearOpMode implements SensorEventListener {
    // Gyro sensor
    private SensorManager sensorManager;
    private Sensor gyroSensor;
    private int sensorType = Sensor.TYPE_GYROSCOPE;
    private static final float nanoSecondsToSeconds = 1.0f / 1000000000.0f;
    private float lastGyroTimestamp = 0;
    private float heading = 0; // In radians
    private float headingDegrees = 0; // In degrees (use in autonomous flow)
    // Autonomous constants
    private final double drivePower = 0.4;
    private final double driveMinPower = 0.2;
    private final double turnPower = 0.4;
    private final int step1Distance = 2000;
    private final int step2Distance = 6200;

    protected enum Alliance {
        Red, Blue
    }
    protected enum Direction {
        Right, Left, Forward, Backward
    }

    public void initialize() throws InterruptedException {
        DragonoidsGlobal.init(hardwareMap);
        DragonoidsGlobal.stopAll();
        // Reset drive motor encoders
        DragonoidsGlobal.resetDriveMotors();
        waitOneFullHardwareCycle();
        DragonoidsGlobal.engageDriveMotors();
        // Set up the gyro sensor
        this.sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        this.gyroSensor = sensorManager.getDefaultSensor(this.sensorType);
        if (this.gyroSensor != null) {
            this.sensorManager.registerListener(this, this.gyroSensor, SensorManager.SENSOR_DELAY_FASTEST);
        }
        else {
            telemetry.addData("Error", "Gyroscope sensor not found");
        }
    }
    protected void outputTelemetry() {
        telemetry.addData("Heading", headingDegrees);
        telemetry.addData("Right Two", DragonoidsGlobal.rightTwo.getCurrentPosition());
        telemetry.addData("Left  Two", DragonoidsGlobal.leftTwo.getCurrentPosition());
    }
    // For gyro sensor data
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != this.sensorType) return;

        final float dT = (event.timestamp - lastGyroTimestamp) * nanoSecondsToSeconds;
        if (lastGyroTimestamp != 0) {
            heading += (dT * event.values[2]);
            headingDegrees = (float) Math.toDegrees(heading);
        }
        lastGyroTimestamp = event.timestamp;

        this.outputTelemetry();
    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        if (sensor.getType() != this.sensorType) return;

        String description;
        switch (accuracy) {
            case SensorManager.SENSOR_STATUS_UNRELIABLE:
                description = "Unreliable";
                break;
            case SensorManager.SENSOR_STATUS_ACCURACY_LOW:
                description = "Low accuracy";
                break;
            case SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM:
                description = "Medium accuracy";
                break;
            case SensorManager.SENSOR_STATUS_ACCURACY_HIGH:
                description = "High accuracy";
                break;
            default:
                description = "None?";
        }
        telemetry.addData("Gyro accuracy changed", String.format("%s (%d)", description, accuracy));
    }

    // Encoder values are negated because forward robot movement results in negative values
    public int getRightEncoderValue() {
        return -1 * DragonoidsGlobal.rightTwo.getCurrentPosition();
    }
    public int getLeftEncoderValue() {
        return -1 * DragonoidsGlobal.leftTwo.getCurrentPosition();
    }

    public void turn(Direction direction, float degrees) throws InterruptedException {
        float startingRotation = this.headingDegrees;
        float targetRotation;

        if (direction == Direction.Right) {
            targetRotation = startingRotation - degrees;
            while (this.headingDegrees > targetRotation) {
                DragonoidsGlobal.setDrivePower(turnPower, -turnPower);
                waitOneFullHardwareCycle();
            }
        }

        if (direction == Direction.Left) {
            targetRotation = startingRotation + degrees;
            while (this.headingDegrees < targetRotation) {
                DragonoidsGlobal.setDrivePower(-turnPower, turnPower);
                waitOneFullHardwareCycle();
            }
        }
        DragonoidsGlobal.stopMotors();
    }
    public void drive(Direction direction, int distance) throws InterruptedException {
        if (direction == Direction.Forward) {
            DragonoidsGlobal.setDrivePower(drivePower, drivePower);
        }
        if (direction == Direction.Backward) {
            DragonoidsGlobal.setDrivePower(-drivePower, -drivePower);
        }
        while ((this.getLeftEncoderValue() + this.getRightEncoderValue()) / 2 < distance) {
            waitOneFullHardwareCycle();
        }
        DragonoidsGlobal.stopMotors();
    }
    public void driveTime(Direction direction, long milliseconds) throws InterruptedException {
        if (direction == Direction.Forward) {
            DragonoidsGlobal.setDrivePower(drivePower, drivePower);
        }
        if (direction == Direction.Backward) {
            DragonoidsGlobal.setDrivePower(-drivePower, -drivePower);
        }
        double startTime = getRuntime();
        double runTime = milliseconds / 1000.0;
        while ((getRuntime() - startTime) < runTime) {
            waitOneFullHardwareCycle();
        }
        DragonoidsGlobal.stopMotors();
    }

    public void autonomous(Alliance alliance) throws InterruptedException {
        this.initialize();
        // Set turning direction for first movements based on alliance color
        // If blue, turn right; if red, turn left
        final Direction turnDirection = (alliance == Alliance.Blue) ? Direction.Right : Direction.Left;
        waitForStart();
        // Run the conveyor backwards so that debris doesn't get caught in the robot
        DragonoidsGlobal.conveyor.setPower(-0.25);
        // Drive forward a bit
        this.drive(Direction.Forward, step1Distance);
        //this.driveTime(Direction.Forward, 1200);
        // Use the phone's IMU to make a precise 45 degree turn
        this.turn(turnDirection, 45);
        // Drive forward to the beacon zone
        this.drive(Direction.Forward, step2Distance);
        //this.driveTime(Direction.Forward, 2500);
        // Turn 45 degrees again
        this.turn(turnDirection, 45);
        // Drive forward to color detection distance
        final int odsBaseValue = DragonoidsGlobal.opticalDistanceSensor.getLightDetectedRaw();
        final int odsThreshold = 10;
        double odsStartTime = getRuntime();
        double maxRunTime = 6; // 6 seconds before watchdog timer kicks in and stops the robot
        while (DragonoidsGlobal.opticalDistanceSensor.getLightDetectedRaw() < (odsBaseValue + odsThreshold) && (getRuntime() - odsStartTime) < maxRunTime) {
            final int lightThreshold = 300;
            if (DragonoidsGlobal.lightSensor.getLightDetectedRaw() < lightThreshold) {
                // Line wasn't found
            }
            else {
                // Line is underneath; continue in a straight line
                DragonoidsGlobal.setDrivePower(driveMinPower, driveMinPower);
            }
            waitOneFullHardwareCycle();
        }
        DragonoidsGlobal.stopMotors();
        // Deposit climbers into bucket
        DragonoidsGlobal.autonomousClimbers.setPosition(0.0);
        sleep(1000);
        DragonoidsGlobal.autonomousClimbers.setPosition(1.0);
        // Detect which button of the beacon to press
        final float beaconScanArc = 10; // The size of the arc (in degrees) that the robot will turn to scan the beacon's colors
        final float beaconPressArc = 5; // The number of degrees the robot will turn (left or right) to press the a beacon button
        final int beaconPressDistance = 500; // The distance the robot will travel to press a beacon button
        Alliance leftBeaconColor = null;
        Alliance rightBeaconColor = null;
        this.turn(Direction.Left, beaconScanArc / 2);
        waitOneFullHardwareCycle();
        leftBeaconColor = (DragonoidsGlobal.colorSensor.red() > DragonoidsGlobal.colorSensor.blue()) ? Alliance.Red : Alliance.Blue;
        this.turn(Direction.Right, beaconScanArc);
        waitOneFullHardwareCycle();
        rightBeaconColor = (DragonoidsGlobal.colorSensor.red() > DragonoidsGlobal.colorSensor.blue()) ? Alliance.Red : Alliance.Blue;
        this.turn(Direction.Left, beaconScanArc / 2);
        // Press the correct button on the beacon
        if (leftBeaconColor != rightBeaconColor) {
            // Only press a button if a difference in colors is detected
            Direction beaconTurnDirection = null;
            if (alliance == leftBeaconColor) {
                beaconTurnDirection = Direction.Left;
            }
            else if (alliance == rightBeaconColor) {
                beaconTurnDirection = Direction.Right;
            }
            else {
                return;
            }
            this.turn(beaconTurnDirection, beaconPressArc);
            // Drive forward to press the button
            this.drive(Direction.Forward, beaconPressDistance);
            sleep(500);
            this.drive(Direction.Backward, beaconPressDistance);
        }

        // Drive forward or extend arm to push the correct button

        // Deposit climbers in the bucket behind the beacon

        // Reverse out of the beacon area (or turn 180 degrees and then drive forward)

        // Turn -45 degrees

        // Drive forward as far as possible up the mountain

        // Use the "churro grabbers" to gain more traction and hoist the robot up the
        // remaining portion of the mountain after the normal wheels begin to slip

        DragonoidsGlobal.stopAll();
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}