package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Right (specimen)", group="Linear OpMode")
//@Disabled
public class Auton extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public Sett robot = new Sett();
    public DriveOld chassis;
    public Imu imu;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private void driveTask(){
        while(opModeIsActive()){
            chassis.getEncoderPositionTrackTask();
            chassis.drivePwrTask();
            sleep(10);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = dashboard.getTelemetry();

        robot.init(hardwareMap);

        imu = new Imu(robot, dashboard, packet);
        chassis = new DriveOld(robot, dashboard, packet, imu);
//        auton = new AutonUtil(robot, arms, drive, odom, imu);
//
//        auton.resetPosition();
//        auton.setCoordinates(18, 21.5, -180);
//        arms.reset();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Thread driveThread = new Thread(this::driveTask);

        waitForStart();

        // AUTON STARTS HERE
        runtime.reset();
        driveThread.start();
        // AUTON ENDS HERE
    }
}
