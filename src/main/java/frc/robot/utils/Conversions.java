package frc.robot.utils;

import frc.robot.Constants.SwerveModuleConstants.MK4I_L2;

/**
 * 
 */
public class Conversions {
    /**
     * 
     */
    public static double degreesTo(double degrees, double gear_ratio) {
        return degrees / (360.0 / (gear_ratio * MK4I_L2.TICKS_PER_ROTATION));
    }
    
    /**
     * 
     */
    public static double toDegrees(double position, double gear_ratio) {
        return position * (360.0 / (gear_ratio * MK4I_L2.TICKS_PER_ROTATION));
    }

    /**
     * 
     */
    public static double toMeters(double position, double gear_ratio) {
        double motorRotations = position / MK4I_L2.TICKS_PER_ROTATION;
        double wheelRotations = motorRotations / gear_ratio;

        return wheelRotations * MK4I_L2.WHEEL_CIRCUMFERENCE;
    }

    /**
     * 
     */
    public static double toMPS(double velocity, double gear_ratio) {
        double wheelRPM = toRPM(velocity, gear_ratio);
        return wheelRPM * MK4I_L2.WHEEL_CIRCUMFERENCE / 60;
    }

    /**
     * 
     */
    public static double toRPM(double velocity, double gear_ratio) {
        double motorRPM = velocity * 600.0 / MK4I_L2.TICKS_PER_ROTATION;

        return motorRPM / gear_ratio;
    }
}
