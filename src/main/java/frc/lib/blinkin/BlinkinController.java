package frc.lib.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * 
 */
public class BlinkinController {
    private static final int CHANNEL_ID = 9;
    private static final Spark spark = new Spark(CHANNEL_ID);

    /**
     * 
     */
    public static void set(IBlinkinPreset preset) {
        set(preset.getValue());
    }

    /**
     * 
     */
    public static void set(double value) {
        spark.set(value);
    }
}
