package frc.robot.modules.gyro;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/**
 * 
 */
public class GyroModuleNavx extends GyroModule {
    private final AHRS gyro;

    /**
     * 
     */
    public GyroModuleNavx() {
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.gyro.reset();
    }

    @Override
    public void update() {
        setAccleration(new double[] { this.gyro.getWorldLinearAccelX(), this.gyro.getWorldLinearAccelY() });
        setIsConnected(this.gyro.isConnected());
        setPitchDeg(this.gyro.getPitch());
        setPositionDeg(this.gyro.getYaw());
        setVelocityDegPerSec(this.gyro.getGyroFullScaleRangeDPS());
    }
}
