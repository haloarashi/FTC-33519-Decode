package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field-Centric Joystick Control", group="Linear OpMode")
public class Field_Centric_Joystick_Control extends LinearOpMode {


    // ===== Motors =====
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotorEx getonShooter, getShooter ;
    private DcMotor intakeone, intaketwo;
    // ===== Pinpoint =====
    private GoBildaPinpointDriver pinpoint;

    // ===== Heading lock =====
    private double headingBearing = 0;
    private double headingOffset = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // ===== PID gains (TUNE) =====
    private static final double kP = 2.0;
    private static final double kD = 0.15;

    @Override
    public void runOpMode() {

        // ===== Motors =====
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "LF");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "LB");
        frontRightDrive = hardwareMap.get(DcMotor.class, "RF");
        backRightDrive  = hardwareMap.get(DcMotor.class, "RB");
        getonShooter = hardwareMap.get(DcMotorEx.class, "S1");
        getShooter = hardwareMap.get(DcMotorEx.class,"S2");
        intakeone = hardwareMap.get(DcMotor.class,"I1");
        intaketwo = hardwareMap.get(DcMotor.class,"I2");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        getonShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        getShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeone.setDirection(DcMotorSimple.Direction.FORWARD);
        intaketwo.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        getonShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        getShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // ===== Pinpoint =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        // Initialize heading
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        headingBearing = getHeading();

        while (opModeIsActive()) {

            // ===== Update heading ONLY =====
            pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

            // ===== Reset heading =====
            if (gamepad1.y) {
                headingOffset = getRawHeading();
                headingBearing = 0;

            }

            // ===== Joystick =====
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yawInput = gamepad1.right_stick_x;

            // ===== Heading lock PID =====
            double yaw;
            double heading = getHeading();

            if (Math.abs(yawInput) > 0.05) {
                yaw = yawInput;
                headingBearing = heading;
                lastError = 0;
                pidTimer.reset();
            } else {
                double error = angleWrap(headingBearing - heading);
                double dt = pidTimer.seconds();
                double derivative = (error - lastError) / dt;

                yaw = kP * error + kD * derivative;

                lastError = error;
                pidTimer.reset();
            }

            // ===== Field-centric transform =====
            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);

            double x = lateral * cosA - axial * sinA;
            double y = lateral * sinA + axial * cosA;

            // ===== Mecanum math =====
            double FL = x - y+ yaw;
            double FR = x - y - yaw;
            double BL = x + y + yaw;
            double BR = x + y - yaw;

            // ===== Normal Speed =====
            double max = Math.max(Math.abs(FL),
                    Math.max(Math.abs(FR),
                            Math.max(Math.abs(BL), Math.abs(BR))));
            if (max > 1.0) {
                FL /= max;
                FR /= max;
                BL /= max;
                BR /= max;
            }

            // ===== Slow mode =====
            if (gamepad1.left_trigger <= 1.0) {
                FL *= 0.5; FR *= 0.5; BL *= 0.5; BR *= 0.5;
            }
            double getonShootPower =(0);
            double getShootPower =(0);
            double intakeonePower =(1);
            double intaketwoPower = (1);
            if (gamepad1.x){
                getonShootPower = 300;
                getShootPower = 300;//轉速
                intakeonePower=(0.8);
                intaketwoPower=(0.8);
            }else{
                getonShootPower=(0);
                getShootPower=(0);
                intakeonePower=(0);
                intaketwoPower=(0);
            }

            intakeone.setPower(intakeonePower);
            intaketwo.setPower(intaketwoPower);
            getonShooter.setVelocity(getonShootPower, AngleUnit.DEGREES);
            getShooter.setVelocity(getShootPower, AngleUnit.DEGREES);

            frontLeftDrive.setPower(FL);
            frontRightDrive.setPower(FR);
            backLeftDrive.setPower(BL);
            backRightDrive.setPower(BR);

            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Target (deg)", Math.toDegrees(headingBearing));
            telemetry.update();
        }
    }

    // ===== Heading helpers =====
    public double getRawHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public double getHeading() {
        return angleWrap(getRawHeading() - headingOffset);
    }


    private double angleWrap(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}