package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PID {
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double error = 0;
    double startI = 0;
    double settleError = 0;
    double settleTime = 0;
    double timeout = 0;
    double accumulatedError = 0;
    double previousError = 0;
    double output = 0;
    double timeSpentSettled = 0;
    double timeSpentRunning = 0;

    public PID(double kp, double ki, double kd, double startI){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.startI = startI;
    }

    public PID(double error, double kp, double ki, double kd, double startI){
        this.error = error;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.startI = startI;
    }

    public PID(double error, double kp, double ki, double kd, double startI,
               double settleError, double settleTime, double timeout){
        this.error = error;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.startI = startI;

        this.settleError = settleError;
        this.settleTime = settleTime;
        this.timeout = timeout;
    }

    public double compute(double error){
        if (abs(error) < startI){
            accumulatedError +=error;
        }
        // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
        if ((error>0 && previousError <0)||(error < 0 && previousError > 0)){
            accumulatedError = 0;
        }

        output = kp * error + ki * accumulatedError + kd * (error - previousError);

        previousError = error;

        if(abs(error) < settleError){
            timeSpentSettled +=10;
        } else {
            timeSpentSettled = 0;
        }

        timeSpentRunning +=10;

        return output;
    }

    public boolean isSettled(){
        if (timeSpentRunning > timeout && timeout != 0){
            return true;
        }
        if (timeSpentSettled > settleTime){
            return true;
        }
        return false;
    }

    public void resetPID(){
        accumulatedError = 0;
        previousError = 0;
        timeSpentSettled = 0;
        timeSpentRunning = 0;
    }

    public void setPID(double kp, double ki, double kd, double startI){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.startI = startI;
    }
}
