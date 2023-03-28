package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

/**
 * 
 */
public class PneumaticSubsystem extends SubsystemBase {
    private final int PRESSURE_MIN = 80;
    private final int PRESSURE_MAX = 120;

    private final PneumaticHub pneumaticHub;

    /**
     * 
     */
    public PneumaticSubsystem() {
        this.pneumaticHub = new PneumaticHub(PneumaticConstants.HUB_ID);
        this.pneumaticHub.enableCompressorAnalog(PRESSURE_MIN, PRESSURE_MAX);
        this.pneumaticHub.getModuleNumber();
    }
    
    /**
     * 
     */
    public DoubleSolenoid getDoubleSolenoid(int forward_channel, int reverse_channel) {
        return this.pneumaticHub.makeDoubleSolenoid(forward_channel, reverse_channel);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Pneumatics/Model", this.pneumaticHub.getModuleNumber());
        Logger.getInstance().recordOutput("Pneumatics/Config", this.pneumaticHub.getCompressorConfigType().toString());
        Logger.getInstance().recordOutput("Pneumatics/Current", this.pneumaticHub.getCompressorCurrent());
        Logger.getInstance().recordOutput("Pneumatics/5VRegulatedVoltage", this.pneumaticHub.get5VRegulatedVoltage());
        Logger.getInstance().recordOutput("Pneumatics/InputVoltage", this.pneumaticHub.getInputVoltage());
        Logger.getInstance().recordOutput("Pneumatics/Brownout Fault", this.pneumaticHub.getFaults().Brownout);
        Logger.getInstance().recordOutput("Pneumatics/Hardware Fault", this.pneumaticHub.getFaults().HardwareFault);
        Logger.getInstance().recordOutput("Pneumatics/Compressor Open Fault", this.pneumaticHub.getFaults().CompressorOpen);
        Logger.getInstance().recordOutput("Pneumatics/Compressor Over Current Fault", this.pneumaticHub.getFaults().CompressorOverCurrent);
        
        for (int channel_id : PneumaticConstants.IN_USE_CHANNELS) {
            Logger.getInstance().recordOutput("Pneumatics/AnalogVoltage [" + channel_id + "]", this.pneumaticHub.getAnalogVoltage(channel_id));
            Logger.getInstance().recordOutput("Pneumatics/Pressure [" + channel_id + "]", this.pneumaticHub.getPressure(channel_id));

            boolean fault = false;
            switch (channel_id) {
                case 12: fault = this.pneumaticHub.getFaults().Channel12Fault; break;
                case 13: fault = this.pneumaticHub.getFaults().Channel13Fault; break;
                case 14: fault = this.pneumaticHub.getFaults().Channel14Fault; break;
                case 15: fault = this.pneumaticHub.getFaults().Channel15Fault; break;
            }
            Logger.getInstance().recordOutput("Pneumatics/Fault [" + channel_id + "]", fault);
        }
    }
}
