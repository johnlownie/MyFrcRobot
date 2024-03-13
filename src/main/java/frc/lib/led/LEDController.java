package frc.lib.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {
    // Constants
    private static final int CHANNEL_ID = 9;
    private static final int STRIP_LENGTH = 250;

    // Hardware
    private static final AddressableLED addressableLED = new AddressableLED(CHANNEL_ID);
    private static final AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(STRIP_LENGTH);

    /**
     * 
     */
    public static void set(Color color) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setLED(i, color);
        }

        addressableLED.setData(addressableLEDBuffer);
    }
    
    /**
     * 
     */
    public static void set(int r, int g, int b) {
        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setRGB(i, r, g, b);
        }

        addressableLED.setData(addressableLEDBuffer);
    }

    /**
     * 
     */
    public static void start() {
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);

        addressableLED.start();
    }
}
