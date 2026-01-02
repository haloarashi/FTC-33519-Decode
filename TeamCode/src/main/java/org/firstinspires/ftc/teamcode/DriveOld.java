//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Math.abs;
//import static java.lang.Math.cos;
//import static java.lang.Math.pow;
//import static java.lang.Math.sin;
//import static java.lang.Math.sqrt;
//import static java.lang.Math.toRadians;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//public class DriveOld {
////    private RobotConfig robot;
//
//    private DcMotorEx leftFrontDrive;
//    private DcMotorEx rightFrontDrive;
//    private DcMotorEx leftBackDrive;
//    private DcMotorEx rightBackDrive;
//
//    private GoBildaPinpointDriver pinpoint;
//
////    private Odom odom;
////    private Imu imu;
//
////    public static double DRIVE_ACCEL_SCALAR = 1.4;
//
//    public static double DRIVE_KD = 0;
//    public static double DRIVE_KI = 0;
//    public static double DRIVE_KP = 0.02;
////    public static double DRIVE_KP_SMALL = 0.02;
//    public static double DRIVE_MAX_PCT = 0.8;
//    public static double DRIVE_MIN_PCT = 0.2; // dont lower this past 0.1
//    public static double DRIVE_SETTLE_ERROR = 5; // originally 4
//    public static double DRIVE_SETTLE_TIME = 20;
//    public static double DRIVE_STARTI = 0;
//    public static double DRIVE_TIMEOUT = 2000;
////    public static double DRIVE_SLOW_THRESHOLD = 50;
//
////    public static double MIN_ACCEL_HEADING = 5;
////    public static double SLOW_TURN_MAX_PCT = 0.3;
//
//    public static double TURN_KD = 0;
//    public static double TURN_KI = 0;
//    public static double TURN_KP = 0.018;
////    public static double TURN_KP_SMALL = 0.02;
//    public static double TURN_MAX_PCT = 0.4;
//    public static double TURN_MIN_PCT = 0.15; // dont lower this past 0.1
//    public static double TURN_SETTLE_ERROR = 4;
//    public static double TURN_SETTLE_TIME = 20;
//    public static double TURN_STARTI = 0;
//    public static double TURN_TIMEOUT = 2000;
////    public static double TURN_SLOW_THRESHOLD = 90;
//
////    public static double ACCEL_THRESHOLD = 50; // do not perform gradual accel if initDriveError < ACCEL_THRESHOLD
////
////    public static double EARLY_EXIT_DRIVE = 30;
////    public static double EARLY_EXIT_HEADING = 45;
//
//    public static double HEADING_KD = 0;
//    public static double HEADING_KI = 0;
//    public static double HEADING_KP = 0.02;
//    public static double HEADING_STARTI = 0;
//
//    private FtcDashboard dashboard;
//    private TelemetryPacket packet;
//
//    public double xMotorPos = 0;
//    public double yMotorPos = 0;
//
//    private double xPwr = 0;
//    private double yPwr = 0;
//    private double turnPwr = 0;
//    private boolean isTurning = false;
//
////    public Drive(RobotConfig robot , FtcDashboard dashboard, TelemetryPacket packet, Imu imu){
//////        this.robot = robot;
////        this.dashboard = dashboard;
////        this.packet = packet;
//////
//////        this.odom = odom;
//////        this.imu = imu;
//////        this.usesOtos = usesOtos;
//////        if(!usesOtos){
//////            odom.encoderSetup(7.62, 1, 16.5, 7.62);
//////        }
//////        odomConstants();
////    }
//
//    public DriveOld(DcMotorEx lf, DcMotorEx rf, DcMotorEx rb, DcMotorEx lb, GoBildaPinpointDriver pin){
//        leftFrontDrive = lf;
//        leftBackDrive = lb;
//        rightFrontDrive = rf;
//        rightBackDrive = rb;
//
//        pinpoint = pin;
//    }
//
//    public void setTurnConstants(double TURN_MAX_PCT, double kp, double ki, double kd, double startI){
//        this.TURN_MAX_PCT = TURN_MAX_PCT;
//        TURN_KP = kp;
//        TURN_KI = ki;
//        TURN_KD = kd;
//        TURN_STARTI = startI;
//    }
//
//    public void setDriveConstants(double DRIVE_MAX_PCT, double kp, double ki, double kd, double startI){
//        this.DRIVE_MAX_PCT = DRIVE_MAX_PCT;
//        DRIVE_KP = kp;
//        DRIVE_KI = ki;
//        DRIVE_KD = kd;
//        DRIVE_STARTI = startI;
//    }
//
//    public void setTurnExitConditions(double TURN_SETTLE_ERROR, double TURN_SETTLE_TIME, double TURN_TIMEOUT){
//        this.TURN_SETTLE_ERROR = TURN_SETTLE_ERROR;
//        this.TURN_SETTLE_TIME = TURN_SETTLE_TIME;
//        this.TURN_TIMEOUT = TURN_TIMEOUT;
//    }
//
//    public void setDriveExitConditions(double DRIVE_SETTLE_ERROR, double DRIVE_SETTLE_TIME, double DRIVE_TIMEOUT){
//        this.DRIVE_SETTLE_ERROR = DRIVE_SETTLE_ERROR;
//        this.DRIVE_SETTLE_TIME = DRIVE_SETTLE_TIME;
//        this.DRIVE_TIMEOUT = DRIVE_TIMEOUT;
//    }
//
//    public void driveStop(){
//        xPwr = 0;
//        yPwr = 0;
//        turnPwr = 0;
//
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//    }
//
//    public void robotCentricTeleDrive(Gamepad gamepad){
//        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//        yPwr = -gamepad.left_stick_y;  // Note: pushing stick forward gives negative value
//        xPwr =  gamepad.left_stick_x;
//        turnPwr =  gamepad.right_stick_x * 0.5;
//
////        drivePower(xPwr, yPwr, turnPwr * 0.5);
//    }
//
//    private PID headingPID; // for field-centric tele drive
//    private double targetHeading;
//
//    public void init(){
//        headingPID = new PID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_STARTI);
//        targetHeading = pinpoint.getHeading(AngleUnit.DEGREES);
//    }
//
//    public void fieldCentricTeleDrive(Gamepad gamepad){
//        // ===== Inputs =====
//        double axial   = -gamepad.left_stick_y;
//        double lateral =  gamepad.left_stick_x;
//        double yawInput = gamepad.right_stick_x;
//        double turnPwr;
//        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
//
//        // Maintains heading if driver isn't turning the chassis
//        boolean driverControlsTurn = abs(yawInput) > 0.05;
//        if (driverControlsTurn) {
//            turnPwr = yawInput;
//            targetHeading = currentHeading;
//            headingPID.resetPID();
//        } else {
//            double error = Util.reduceNegative180To180(targetHeading - currentHeading);
//            turnPwr = headingPID.compute(error);
//            turnPwr = Util.clamp(turnPwr, -TURN_MAX_PCT, TURN_MAX_PCT);
//            turnPwr = Util.clampMinPct(turnPwr, TURN_MIN_PCT);
//        }
//
//        // ===== Field-centric transform =====
//        double cosA = Math.cos(toRadians(-currentHeading));
//        double sinA = Math.sin(toRadians(-currentHeading));
//
//        double xPwr = lateral * cosA - axial * sinA;
//        double yPwr = lateral * sinA + axial * cosA;
//
//        drivePower(xPwr, yPwr, turnPwr);
//    }
//
//    public void drivePower(double xPwr, double yPwr, double turnPwr){
//        double leftFrontPower  = yPwr + xPwr - turnPwr;
//        double rightFrontPower = yPwr - xPwr + turnPwr;
//        double leftBackPower   = yPwr - xPwr - turnPwr;
//        double rightBackPower  = yPwr + xPwr + turnPwr;
//
//        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
//        max = Math.max(max, abs(leftBackPower));
//        max = Math.max(max, abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        }
//
////         This is test code.
////         Uncomment the following code to test your motor directions.
////         Each button should make the corresponding motor run FORWARD.
////
////            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
////            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
////            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
////            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }
//
//    /**
//     *
//     * @param target desired angle in degrees (any angle works as reduceNegative180To180() is called)
//     */
//    public void turnToAngle(double target) throws InterruptedException {
//        PID turnPID = new PID(TURN_KP, TURN_KI, TURN_KD, TURN_STARTI);
//        targetHeading = Util.reduceNegative180To180(target);
//
//        while(!turnPID.isSettled()){
//            double error =  Util.reduceNegative180To180(target - pinpoint.getHeading());
//
//            double output = turnPID.compute(error);
//            output = Util.clamp(output, -TURN_MAX_PCT, TURN_MAX_PCT);
//            output = Util.clampMinPct(output, TURN_MIN_PCT);
//
//            isTurning = true;
//            turnPwr = output;
////             drivePower(xPwr_to_maintain, yPwr_to_maintain, output);
//        }
//        isTurning = false; // or basically "isTurning = turnPID.isSettled();"
//    }
//
//    /**
//     *
//     * @param target desired target distance in inches
//     * @param direction desired direction vector in degrees (any angle works as angleTo360() is called)
//     */
//    public void driveDistance(double target, double direction) throws InterruptedException {
//        PID drivePID = new PID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_STARTI);
//        PID headingPID = new PID(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_STARTI);
//
//        direction = Util.angleTo360(direction);
//        direction = toRadians(direction);
//
//        double initY = yMotorPos;
//        double initX = xMotorPos;
//
//        while(!drivePID.isSettled()){
//            double dist_travelled = sqrt(pow(yMotorPos - initY, 2) + pow(xMotorPos - initX, 2));
//            double driveError = target - dist_travelled;
//            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
//
//            double output = drivePID.compute(driveError);
//            output = Util.clamp(output, -DRIVE_MAX_PCT, DRIVE_MAX_PCT);
//            output = Util.clampMinPct(output, DRIVE_MIN_PCT);
//
//            xPwr = output * cos(direction);
//            yPwr = output * sin(direction);
////            drivePower(output * cos(direction), output * sin(direction), turnPwrToMaintain);
//
//            if(!isTurning){
//                double headingError = Util.reduceNegative180To180(targetHeading - currentHeading);
//                turnPwr = headingPID.compute(headingError);
//                turnPwr = Util.clamp(turnPwr, -TURN_MAX_PCT, TURN_MAX_PCT);
//                turnPwr = Util.clampMinPct(turnPwr, TURN_MIN_PCT);
//            }
//        }
//    }
//
//
//
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
//
//    public void drivePwrTask(){
//        drivePower(xPwr, yPwr, turnPwr);
//    }
//
//}
