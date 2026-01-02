package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/* Front arm up-down tilt: b
 * Front claw left-right tilt: x
 * Front claw open-close: a
 * Front arm extend: y
 *
 * Back arm up-down: dpad_right
 * Back claw left-right tilt: dpad_left
 * Back claw open-close: dpad_down
 * Back arm extend: dpad_up
 * Back arm left-right tilt: lb/rb
 */


public class Sett {
    DcMotorEx leftFrontDrive;
    DcMotorEx leftBackDrive;
    DcMotorEx rightFrontDrive;
    DcMotorEx rightBackDrive;

    GoBildaPinpointDriver pinpoint;

//    CRServo intake;
//
//    Servo frontArmRight;
//    Servo frontArmLeft;
//    Servo frontArmRightExtend;
//    Servo frontArmLeftExtend;
//
//    Servo backClawOpenClose;
//
//    Servo backArmExtend;
//    Servo backArmRight;
//    Servo backArmLeft;
//
//    DcMotor LiftLeft;
//    DcMotor LiftRight;
//
//    SparkFunOTOS otos;

    IMU imu;
    RevHubOrientationOnRobot imuOrientation;
    RevHubOrientationOnRobot.LogoFacingDirection imuLogoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection  imuUsbDirection;
    YawPitchRollAngles orientation;

    public void init(HardwareMap hardwareMap){
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rb");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "idk");

//        intake = hardwareMap.get(CRServo.class, "in");
//
//        frontArmRight = hardwareMap.get(Servo.class, "frontArmRight");
//        frontArmLeft = hardwareMap.get(Servo.class, "frontArmLeft");
//        frontArmRightExtend = hardwareMap.get(Servo.class, "frontArmRightExtend");
//        frontArmLeftExtend = hardwareMap.get(Servo.class, "frontArmLeftExtend");
//
//        backClawOpenClose = hardwareMap.get(Servo.class, "backClawOpenClose");
//
//        backArmExtend = hardwareMap.get(Servo.class, "backArmExtend");
//        backArmRight = hardwareMap.get(Servo.class, "backArmRight");
//        backArmLeft = hardwareMap.get(Servo.class, "backArmLeft");
//
//        LiftLeft = hardwareMap.get(DcMotor.class,"LiftLeft");
//        LiftRight = hardwareMap.get(DcMotor.class,"LiftRight");

//        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        imu = hardwareMap.get(IMU.class, "imu");

        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

//        frontArmLeft.setDirection(Servo.Direction.REVERSE);
//        backArmRight.setDirection(Servo.Direction.REVERSE);
//        frontArmLeftExtend.setDirection(Servo.Direction.REVERSE);
//        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        imuLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        imuUsbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//
        imuOrientation = new RevHubOrientationOnRobot(imuLogoDirection, imuUsbDirection);

//        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        LiftRight.setTargetPosition(0);
//        LiftLeft.setTargetPosition(0);
//        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        LiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu.initialize(new IMU.Parameters(imuOrientation));

        orientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    }

    /**
     *
     * @return robot heading in degrees [0, 360)
     */
    public double getHeading(){
        orientation = imu.getRobotYawPitchRollAngles();
        return Util.angleTo360(orientation.getYaw(AngleUnit.DEGREES));
    }

}
