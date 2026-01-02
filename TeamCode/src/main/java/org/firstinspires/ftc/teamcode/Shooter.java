package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {

    public void idk(){
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

        intakeone.setPower(intakeonePower);
        intaketwo.setPower(intaketwoPower);
        getonShooter.setVelocity(getonShootPower, AngleUnit.DEGREES);
        getShooter.setVelocity(getShootPower, AngleUnit.DEGREES);


        // ===== Field-centric transform =====
        double cosA = Math.cos(-heading);
        double sinA = Math.sin(-heading);

        double x = lateral * cosA - axial * sinA;
        double y = lateral * sinA + axial * cosA;
    }
}
