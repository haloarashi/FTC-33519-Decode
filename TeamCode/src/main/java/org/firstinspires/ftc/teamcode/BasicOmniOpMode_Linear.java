package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    Sett robot = new Sett();
    Drive chassis;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        chassis = new Drive(robot.leftFrontDrive, robot.rightFrontDrive, robot.rightBackDrive, robot.leftBackDrive, robot.pinpoint, robot.imu);
        chassis.init();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.pinpoint.update(ONLY_UPDATE_HEADING);
            chassis.fieldCentricTeleDrive(gamepad1);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            sleep(10);
        }
    }}
