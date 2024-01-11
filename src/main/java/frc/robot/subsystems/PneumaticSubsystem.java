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
        Logger.recordOutput("Subsystems/Pneumatics/Model", this.pneumaticHub.getModuleNumber());
        Logger.recordOutput("Subsystems/Pneumatics/Config", this.pneumaticHub.getCompressorConfigType().toString());
        Logger.recordOutput("Subsystems/Pneumatics/Current", this.pneumaticHub.getCompressorCurrent());
        Logger.recordOutput("Subsystems/Pneumatics/5VRegulatedVoltage", this.pneumaticHub.get5VRegulatedVoltage());
        Logger.recordOutput("Subsystems/Pneumatics/InputVoltage", this.pneumaticHub.getInputVoltage());
        Logger.recordOutput("Subsystems/Pneumatics/Brownout Fault", this.pneumaticHub.getFaults().Brownout);
        Logger.recordOutput("Subsystems/Pneumatics/Hardware Fault", this.pneumaticHub.getFaults().HardwareFault);
        Logger.recordOutput("Subsystems/Pneumatics/Compressor Open Fault", this.pneumaticHub.getFaults().CompressorOpen);
        Logger.recordOutput("Subsystems/Pneumatics/Compressor Over Current Fault", this.pneumaticHub.getFaults().CompressorOverCurrent);
        
        for (int channel_id : PneumaticConstants.IN_USE_CHANNELS) {
            Logger.recordOutput("Subsystems/Pneumatics/AnalogVoltage [" + channel_id + "]", this.pneumaticHub.getAnalogVoltage(channel_id));
            Logger.recordOutput("Subsystems/Pneumatics/Pressure [" + channel_id + "]", this.pneumaticHub.getPressure(channel_id));

            boolean fault = false;
            switch (channel_id) {
                case 12: fault = this.pneumaticHub.getFaults().Channel12Fault; break;
                case 13: fault = this.pneumaticHub.getFaults().Channel13Fault; break;
                case 14: fault = this.pneumaticHub.getFaults().Channel14Fault; break;
                case 15: fault = this.pneumaticHub.getFaults().Channel15Fault; break;
            }
            Logger.recordOutput("Subsystems/Pneumatics/Fault [" + channel_id + "]", fault);
        }
    }
}
