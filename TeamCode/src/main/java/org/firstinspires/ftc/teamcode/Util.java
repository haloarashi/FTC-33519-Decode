package org.firstinspires.ftc.teamcode;

public class Util {
    public static double clamp(double input, double min, double max){
        if(input > max){
            return max;
        }
        if(input < min){
            return min;
        }
        return input;
    }

    /**
     * Converts an angle to an equivalent one in the range [-180, 180).
     *
     * @param angle The angle to be reduced in degrees.
     * @return Reduced angle, right is positive and left is negative.
     */
    public static double reduceNegative180To180(double angle) {
        while(!(angle >= -180 && angle < 180)) {
            if( angle < -180 ) { angle += 360; }
            if(angle >= 180) { angle -= 360; }
        }
        return(angle);
    }

    /**
     * Converts an angle to 0~360
     * @param angle in 0~360 or [-180, 180)
     * @return angle in 0~360, increases clockwise
     */
    public static double angleTo360(double angle){
        return (angle + 360) % 360;
    }

    /**
     * Converts an angle to an equivalent one in the range [-90, 90).
     * If the angle has no equivalent, then the angle halfway around
     * the circle is returned.
     *
     * @param angle The angle to be reduced in degrees.
     * @return Reduced angle.
     */
    public static double reduceNegative90To90(double angle) {
        while(!(angle >= -90 && angle < 90)) {
            if(angle < -90) {
                angle += 180;
            }
            if(angle >= 90) {
                angle -= 180;
            }
        }
        return angle;
    }

    /**
     * Brings an output up to the minimum voltage if it's too slow.
     * Used for minimum voltage calculations for movement chaining.
     * Has no effect on 0 voltage output, because how do we know
     * which way it's supposed to be going?
     *
     * @param output The forward output of the drive.
     * @param minPct The minimum output of the drive.
     * @return The output with the minimum applied.
     */
    public static double clampMinPct(double output, double minPct){
        if(output < 0 && output > -minPct){
            return -minPct;
        }
        if(output > 0 && output < minPct){
            return minPct;
        }
        return output;
    }
}