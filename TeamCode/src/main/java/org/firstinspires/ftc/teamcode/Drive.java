package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.reduceNegative180To180;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive {
//    private RobotConfig robot;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    private GoBildaPinpointDriver pinpoint;
//    private IMU imu;
    private Sett robot;

//    private Odom odom;
//    private Imu imu;

//    public static double DRIVE_ACCEL_SCALAR = 1.4;

    public static double DRIVE_KD = 0;
    public static double DRIVE_KI = 0;
    public static double DRIVE_KP = 0.02;
//    public static double DRIVE_KP_SMALL = 0.02;
    public static double DRIVE_MAX_PCT = 0.8;
    public static double DRIVE_MIN_PCT = 0.2; // dont lower this past 0.1
    public static double DRIVE_SETTLE_ERROR = 5; // originally 4
    public static double DRIVE_SETTLE_TIME = 20;
    public static double DRIVE_STARTI = 0;
    public static double DRIVE_TIMEOUT = 2000;
//    public static double DRIVE_SLOW_THRESHOLD = 50;

//    public static double MIN_ACCEL_HEADING = 5;
//    public static double SLOW_TURN_MAX_PCT = 0.3;

    public static double TURN_KD = 0;
    public static double TURN_KI = 0;
    public static double TURN_KP = 0.018;
//    public static double TURN_KP_SMALL = 0.02;
    public static double TURN_MAX_PCT = 0.4;
    public static double TURN_MIN_PCT = 0.15; // dont lower this past 0.1
    public static double TURN_SETTLE_ERROR = 4;
    public static double TURN_SETTLE_TIME = 20;
    public static double TURN_STARTI = 0;
    public static double TURN_TIMEOUT = 2000;
//    public static double TURN_SLOW_THRESHOLD = 90;

//    public static double ACCEL_THRESHOLD = 50; // do not perform gradual accel if initDriveError < ACCEL_THRESHOLD
//
//    public static double EARLY_EXIT_DRIVE = 30;
//    public static double EARLY_EXIT_HEADING = 45;

    public static double HEADING_KD = 0;
    public static double HEADING_KI = 0;
    public static double HEADING_KP = 0.02;
    public static double HEADING_STARTI = 0;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

//    public Drive(RobotConfig robot , FtcDashboard dashboard, TelemetryPacket packet, Imu imu){
////        this.robot = robot;
//        this.dashboard = dashboard;
//        this.packet = packet;
////
////        this.odom = odom;
////        this.imu = imu;
////        this.usesOtos = usesOtos;
////        if(!usesOtos){
////            odom.encoderSetup(7.62, 1, 16.5, 7.62);
////        }
////        odomConstants();
//    }

    public Drive(DcMotorEx lf, DcMotorEx rf, DcMotorEx rb, DcMotorEx lb, GoBildaPinpointDriver pin, Sett robot){
        leftFrontDrive = lf;
        leftBackDrive = lb;
        rightFrontDrive = rf;
        rightBackDrive = rb;

        pinpoint = pin;
        this.robot = robot;
    }

    public void setTurnConstants(double TURN_MAX_PCT, double kp, double ki, double kd, double startI){
        this.TURN_MAX_PCT = TURN_MAX_PCT;
        TURN_KP = kp;
        TURN_KI = ki;
        TURN_KD = kd;
        TURN_STARTI = startI;
    }

    public void setDriveConstants(double DRIVE_MAX_PCT, double kp, double ki, double kd, double startI){
        this.DRIVE_MAX_PCT = DRIVE_MAX_PCT;
        DRIVE_KP = kp;
        DRIVE_KI = ki;
        DRIVE_KD = kd;
        DRIVE_STARTI = startI;
    }

    public void setTurnExitConditions(double TURN_SETTLE_ERROR, double TURN_SETTLE_TIME, double TURN_TIMEOUT){
        this.TURN_SETTLE_ERROR = TURN_SETTLE_ERROR;
        this.TURN_SETTLE_TIME = TURN_SETTLE_TIME;
        this.TURN_TIMEOUT = TURN_TIMEOUT;
    }

    public void setDriveExitConditions(double DRIVE_SETTLE_ERROR, double DRIVE_SETTLE_TIME, double DRIVE_TIMEOUT){
        this.DRIVE_SETTLE_ERROR = DRIVE_SETTLE_ERROR;
        this.DRIVE_SETTLE_TIME = DRIVE_SETTLE_TIME;
        this.DRIVE_TIMEOUT = DRIVE_TIMEOUT;
    }

    public void driveStop(){
//        xPwr = 0;
//        yPwr = 0;
//        turnPwr = 0;

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void robotCentricTeleDrive(Gamepad gamepad){
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double yPwr = -gamepad.left_stick_y;  // Note: pushing stick forward gives negative value
        double xPwr =  gamepad.left_stick_x;
        double turnPwr =  gamepad.right_stick_x * 0.5;

        drivePower(xPwr, yPwr, turnPwr * 0.5);
    }

    private PID headingPID; // for field-centric tele drive
    private double targetHeading;

    public void init(){
        pinpoint.update();
        headingPID = new PID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_STARTI);
        targetHeading = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void fieldCentricTeleDrive(Gamepad gamepad){
        // ===== Inputs =====
        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x;
        double yawInput = gamepad.right_stick_x;
        double turnPwr;
//        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double currentHeading = robot.getHeading();

        // Maintains heading if driver isn't turning the chassis
        boolean driverControlsTurn = abs(yawInput) > 0.05;
        if (driverControlsTurn) {
            turnPwr = yawInput;
            targetHeading = currentHeading;
            headingPID.resetPID();
        } else {
            double error = Util.reduceNegative180To180(targetHeading - currentHeading);
            turnPwr = headingPID.compute(error);
            turnPwr = Util.clamp(turnPwr, -TURN_MAX_PCT, TURN_MAX_PCT);
            turnPwr = Util.clampMinPct(turnPwr, TURN_MIN_PCT);
        }

        // ===== Field-centric transform =====
        double cosA = Math.cos(toRadians(-currentHeading));
        double sinA = Math.sin(toRadians(-currentHeading));

        double xPwr = lateral * cosA - axial * sinA;
        double yPwr = lateral * sinA + axial * cosA;

        drivePower(xPwr, yPwr, turnPwr);
    }

    public void drivePower(double xPwr, double yPwr, double turnPwr){
        double leftFrontPower  = yPwr + xPwr - turnPwr;
        double rightFrontPower = yPwr - xPwr + turnPwr;
        double leftBackPower   = yPwr - xPwr - turnPwr;
        double rightBackPower  = yPwr + xPwr + turnPwr;

        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

//         This is test code.
//         Uncomment the following code to test your motor directions.
//         Each button should make the corresponding motor run FORWARD.
//
//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     *
     * @param target desired angle in degrees (any angle works as reduceNegative180To180() is called)
     */
    public void turnToAngle(double target) throws InterruptedException {
        PID turnPID = new PID(TURN_KP, TURN_KI, TURN_KD, TURN_STARTI);
        targetHeading = Util.reduceNegative180To180(target);

        while(!turnPID.isSettled()){
            pinpoint.update();
            double error =  Util.reduceNegative180To180(target - pinpoint.getHeading(AngleUnit.DEGREES));

            double output = turnPID.compute(error);
            output = Util.clamp(output, -TURN_MAX_PCT, TURN_MAX_PCT);
            output = Util.clampMinPct(output, TURN_MIN_PCT);

//            isTurning = true;
//            turnPwr = output;
             drivePower(0, 0, output);

             sleep(10);
        }
//        isTurning = false; // or basically "isTurning = turnPID.isSettled();"
    }

    /**
     *
     * @param target desired target distance in inches
     * @param direction desired direction vector in degrees (any angle works as angleTo360() is called)
     */
    public void driveDistance(double target, double direction) throws InterruptedException {
        PID drivePID = new PID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_STARTI);
        PID headingPID = new PID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_STARTI);

        direction = Util.angleTo360(direction);
        direction = toRadians(direction);

//        double initY = yMotorPos;
//        double initX = xMotorPos;
        pinpoint.update();
        double initY = pinpoint.getPosition().getY(DistanceUnit.INCH);
        double initX = pinpoint.getPosition().getX(DistanceUnit.INCH);

        while(!drivePID.isSettled()){
            pinpoint.update();
            double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double dist_travelled = sqrt(pow(currentY - initY, 2) + pow(currentX - initX, 2));
            double driveError = target - dist_travelled;
            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            double output = drivePID.compute(driveError);
            output = Util.clamp(output, -DRIVE_MAX_PCT, DRIVE_MAX_PCT);
            output = Util.clampMinPct(output, DRIVE_MIN_PCT);

//            double xPwr = output * cos(direction);
//            double yPwr = output * sin(direction);

//            if(!isTurning){
            double headingError = Util.reduceNegative180To180(targetHeading - currentHeading);
            double turnPwr = headingPID.compute(headingError);
            turnPwr = Util.clamp(turnPwr, -TURN_MAX_PCT, TURN_MAX_PCT);
            turnPwr = Util.clampMinPct(turnPwr, TURN_MIN_PCT);
//            }
            drivePower(output * cos(direction), output * sin(direction), turnPwr);

            sleep(10);
        }
    }



//    public void getEncoderPositionTrackTask(){
//        double rfDeg = rightFrontDrive.getCurrentPosition();
//        double rbDeg = rightBackDrive.getCurrentPosition();
//        double lfDeg = leftFrontDrive.getCurrentPosition();
//        double lbDeg = leftBackDrive.getCurrentPosition();
//
//        yMotorPos = - (rfDeg + rbDeg + lfDeg + lbDeg) * ((91.9/8149 + 91.9/8103 + 91.5/7880) / 3);
//        xMotorPos = (+ rfDeg - rbDeg - lfDeg + lbDeg) * ((84.1/8109 + 84.6/8113 + 84.6/8106) / 3);
////        odom.updatePosition(yPos, xPos, imu.getHeading());
////
////        packet.put("xPos", xPos);
////        packet.put("yPos", yPos);
//    }

//    public void drivePwrTask(){
//        drivePower(xPwr, yPwr, turnPwr);
//    }

public void driveToPose(double targetX, double targetY, double targetHeading,
                        double driveMaxPct, double turnMaxPct, double driveMinPct, double turnMinPct,
                        double driveSettleError, double driveSettleTime, double driveTimeout,
                        double turnSettleError, double turnSettleTime, double turnTimeout,
                        double driveKp, double driveKi, double driveKd, double driveStarti,
                        double turnKp, double turnKi, double turnKd, double turnStarti,
                        double earlyExitDrive, double earlyExitHeading, boolean accel, boolean isConnector) throws InterruptedException {
    double NORMAL_TURN_MAX_PCT = turnMaxPct;

    double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
    double currentY = odom.getPosition(usesOtos).y;
    double currentHeading = Util.angleTo360(odom.getPosition(usesOtos).h);

    double xError = targetX - currentX;
    double yError = targetY - currentY;

    double initDriveError = hypot(xError, yError);
    PID drivePID = new PID(initDriveError, driveKp, driveKi, driveKd, driveStarti, driveSettleError, driveSettleTime, driveTimeout);

    double initTurnError = - reduceNegative180To180(targetHeading - currentHeading); // targetHeading is 0~360
    PID turnPID = new PID(initTurnError, turnKp, turnKi, turnKd, turnStarti, turnSettleError, turnSettleTime, turnTimeout);

    double accelDist = driveMaxPct / driveKp; // The robot will do gradual acceleration for the first accelDist of the movement

    boolean driveReached = drivePID.isSettled();
    boolean turnReached = turnPID.isSettled();

    while(!(drivePID.isSettled() && turnPID.isSettled())){
        encoderPositionTrackTask();

//            // seems unnecessary vvv
//            if(abs(drivePID.error) < DRIVE_SLOW_THRESHOLD){
//                drivePID.kp = DRIVE_KP_SMALL;
//            }
//            else{
//                drivePID.kp = driveKp;
//            }
//
//            if(abs(turnPID.error) < TURN_SLOW_THRESHOLD){
//                turnPID.kp = TURN_KP_SMALL;
//            }
//            else{
//                turnPID.kp = turnKp;
//            }
//            // ^^^

        turnMaxPct = NORMAL_TURN_MAX_PCT;

        currentX = odom.getPosition(usesOtos).x;
        currentY = odom.getPosition(usesOtos).y;
        currentHeading = Util.angleTo360(odom.getPosition(usesOtos).h);

        xError = targetX - currentX;
        yError = targetY - currentY;

        double driveError = hypot(xError, yError);

        double turnError = - reduceNegative180To180(targetHeading - currentHeading);

        if(isConnector){
            if(driveError < earlyExitDrive){
                driveReached = true;
            }
            if(abs(turnError) < earlyExitHeading){
                turnReached = true;
            }

            if(driveReached && turnReached){
                break;
            }
        }

        double driveOutput = drivePID.compute(driveError);
        double turnOutput = turnPID.compute(turnError);
            }
        }

        if(!turnPID.isSettled()){
            if(0 < turnOutput && turnOutput < turnMinPct){
                turnOutput = turnMinPct;
            }
            else if(-turnMaxPct < turnOutput && turnOutput < 0){
                turnOutput = -turnMinPct;
            }
        }
        if(!drivePID.isSettled()){
            if(0 < driveOutput && driveOutput < driveMinPct){
                driveOutput = driveMinPct;
            }
            else if(-driveMaxPct < driveOutput && driveOutput < 0){
                driveOutput = -driveMinPct;
            }
        }

        driveOutput = clamp(driveOutput, -driveMaxPct, driveMaxPct);
        turnOutput = clamp(turnOutput, -turnMaxPct, turnMaxPct);

        double headingError = atan2(yError, xError) - toRadians(odom.getPosition(usesOtos).h);
        drivePower(driveOutput * cos(headingError), driveOutput * sin(headingError), turnOutput);

        packet.put("Turn time", turnPID.timeSpentRunning);
        packet.put("Turn timeout", turnTimeout);
        packet.put("Turn isSettled", turnPID.isSettled());
        packet.put("Turn error", turnError);
        packet.put("Turn error (init)", initTurnError);

        packet.put("Drive time", drivePID.timeSpentRunning);
        packet.put("Drive timeout", driveTimeout);
        packet.put("Drive isSettled", drivePID.isSettled());
        packet.put("Drive error", driveError);

        packet.put("driveOutput", driveOutput);
        packet.put("turnOutput", turnOutput);

        packet.put("driveKp", drivePID.kp);
        packet.put("turnKp", turnPID.kp);

        sleep(10);
    }
}

}
