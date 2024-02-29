package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * 
 */
public class Conversions {
    /**
     * 
     */
    public static double distanceBetween(Pose2d pose1, Pose2d pose2) {
        if (pose1 == null || pose2 == null) return 0.0;
        
        double xDiff = pose2.getX() - pose1.getX();
        double yDiff = pose2.getY() - pose1.getY();

        return Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));
    }

    /**
     * @param wheelMPS      Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param velocityRadiansPerSecond
     * @param circumference  Wheel Circumference: (in Meters)
     * @return Velocity:(in Meters Per Second)
     */
    public static double RADToMPS(double velocityRadiansPerSecond, double circumference) {
        return velocityRadiansPerSecond * (circumference / (2.0 * Math.PI));
    }

    /**
     * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMeters   Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
    
    /**
     * 
     */
    public static double toDegrees(double position, double gear_ratio) {
        return position * (360.0 / (gear_ratio * SwerveModuleConstants.TICKS_PER_ROTATION));
    }

    /**
     * 
     */
    public static double toMeters(double position, double gear_ratio, double circumference) {
        double motorRotations = position / SwerveModuleConstants.TICKS_PER_ROTATION;
        double wheelRotations = motorRotations / gear_ratio;

        return wheelRotations * circumference;
    }

    /**
     * 
     */
    public static double toRPM(double velocity, double gear_ratio) {
        double motorRPM = velocity * 600.0 / SwerveModuleConstants.TICKS_PER_ROTATION;

        return motorRPM / gear_ratio;
    }
}
