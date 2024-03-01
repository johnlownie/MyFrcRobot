package frc.robot.modules.shooter;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * 
 */
public class ShooterModuleSimulator extends ShooterModule {
    /**
     * 
     */
    public ShooterModuleSimulator() {
        super();

        REVPhysicsSim.getInstance().addSparkMax(this.leftMotor, DCMotor.getNEO(1));
    }

    @Override
    public boolean hasNote() {
        return super.hasNote();
    }

    @Override
    public void update() {
        REVPhysicsSim.getInstance().run();

        super.update();
    }
}
