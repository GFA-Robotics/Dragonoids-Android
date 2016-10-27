package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
//Dragonoids Motor Initializer class
public class DragonoidsGlobal {

    // Drive motors
    public static DcMotor rightOne, rightTwo, leftOne, leftTwo;
    // Aux motors
    public static DcMotor conveyor, dispenser;
    // Slider motors
    public static DcMotor leftSlider, rightSlider;
    // Servos
    public static Servo rightClimber, leftClimber, autonomousClimbers, rightClamp, leftClamp;
    // Sensors
    public static ColorSensor colorSensor;
    public static OpticalDistanceSensor opticalDistanceSensor;
    public static LightSensor lightSensor;

    public static void init(HardwareMap hardwareMap) {
        rightOne = hardwareMap.dcMotor.get("rightOneDrive");
        rightTwo = hardwareMap.dcMotor.get("rightTwoDrive");
        leftOne = hardwareMap.dcMotor.get("leftOneDrive");
        leftTwo = hardwareMap.dcMotor.get("leftTwoDrive");

        rightOne.setDirection(DcMotor.Direction.REVERSE);
        leftTwo.setDirection(DcMotor.Direction.REVERSE);

        conveyor = hardwareMap.dcMotor.get("conveyor");
        dispenser = hardwareMap.dcMotor.get("dispenser");

        leftSlider = hardwareMap.dcMotor.get("leftSlider");
        rightSlider = hardwareMap.dcMotor.get("rightSlider");

        leftSlider.setDirection(DcMotor.Direction.REVERSE);

        rightClimber = hardwareMap.servo.get("rightClimber");
        leftClimber = hardwareMap.servo.get("leftClimber");
        autonomousClimbers = hardwareMap.servo.get("autonomousClimbers");
        rightClamp = hardwareMap.servo.get("rightClamp");
        leftClamp = hardwareMap.servo.get("leftClamp");
        leftClamp.setDirection(Servo.Direction.REVERSE);
        resetServos();

        colorSensor = hardwareMap.colorSensor.get("color");
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("distance");
        lightSensor = hardwareMap.lightSensor.get("light");
        // Enable to read reflected light and disable to read emitted light
        enableLEDs(true);
    }
    public static void init(HardwareMap hardwareMap, boolean resetDriveMotors) {
        init(hardwareMap);
        if (resetDriveMotors) {
            // Must call engageDriveMotors() after this to reengage the motors
            resetDriveMotors();
        }
    }

    public static void enableLEDs(boolean state) {
        lightSensor.enableLed(state);
        colorSensor.enableLed(state);
    }

    public static void setDrivePower(double rightPower, double leftPower) {
        setDrivePower(rightPower, leftPower, false);
    }
    public static void setDrivePower(double rightPower, double leftPower, boolean compensate) {
        final double compensation = 0.15;
        if (compensate) {
            rightPower += compensation;
        }
        rightOne.setPower(rightPower);
        rightTwo.setPower(rightPower);
        leftOne.setPower(leftPower);
        leftTwo.setPower(leftPower);
    }

    public static void resetDriveMotors() {
        DragonoidsGlobal.rightTwo.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        DragonoidsGlobal.leftTwo.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
    public static void engageDriveMotors() {
        DragonoidsGlobal.rightTwo.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        DragonoidsGlobal.leftTwo.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }
    public static void resetServos(){
        rightClimber.setPosition(0.0);
        leftClimber.setPosition(0.0);
        autonomousClimbers.setPosition(1.0);
        rightClamp.setPosition(0.0);
        leftClamp.setPosition(0.0);
    }

    public static void stopMotors() {
        setDrivePower(0, 0);
    }
    public static void stopAll() {
        // Stop all motors
        stopMotors();
        conveyor.setPower(0);
        dispenser.setPower(0);
        leftSlider.setPower(0);
        rightSlider.setPower(0);
        resetServos();
    }
}