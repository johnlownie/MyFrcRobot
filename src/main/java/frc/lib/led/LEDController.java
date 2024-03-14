package frc.lib.led;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {
    // Effects
    public static enum Effect {
        SOLID, STROBE
    }

    // Constants
    private static final int CHANNEL_ID = 3;
    private static final int STRIP_LENGTH = 250;

    // Hardware
    private static final AddressableLED addressableLED = new AddressableLED(CHANNEL_ID);
    private static final AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(STRIP_LENGTH);

    // Variables
    private static final Timer timer = new Timer();
    private static TimerTask timerTask;
    private static int ledIndex = 0;

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
    public static void set(Effect effect, Color color) {
        ledIndex = 0;
        if (timerTask != null) timerTask.cancel();

        switch (effect) {
            case STROBE:
                timerTask = new TimerTask() {
                    @Override
                    public void run() {
                        strobe(color);
                    }
                };
        
                timer.scheduleAtFixedRate(timerTask, 500, 10);
                break;
        
            default:
                for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
                    addressableLEDBuffer.setLED(i, color);
                }

                addressableLED.setData(addressableLEDBuffer);
                break;
        }
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

    /**
     * 
     */
    public static void stop() {
        addressableLED.stop();
        timer.cancel();
    }

    /**
     * 
     */
    private static void strobe(Color color) {
        ledIndex ++;
        if (ledIndex >= STRIP_LENGTH) ledIndex = 0;

        for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            if ((i + ledIndex) % 2 == 0) {
                addressableLEDBuffer.setLED(i, color);
            }
            else {
                addressableLEDBuffer.setLED(i, Color.kBlack);
            }
        }

        addressableLED.setData(addressableLEDBuffer);
    }
}
