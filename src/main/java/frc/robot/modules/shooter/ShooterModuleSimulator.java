package frc.robot.modules.shooter;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.util.Timer;

/**
 * 
 */
public class ShooterModuleSimulator extends ShooterModule {
    // used to simulate note intake
    private boolean has_note;
    private Timer timer;

    /**
     * 
     */
    public ShooterModuleSimulator() {
        super();

        REVPhysicsSim.getInstance().addSparkMax(this.leftMotor, DCMotor.getNEO(1));
        this.has_note = false;
        this.timer = new Timer();
    }

    @Override
    public boolean hasNote() {
        return this.has_note;
    }

    @Override
    public void intake() {
        super.intake();

        this.has_note = false;
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void shoot() {
        super.shoot();

        this.has_note = false;
    }

    @Override
    public void update() {
        REVPhysicsSim.getInstance().run();

        if (this.timer.isRunning() && this.timer.hasElapsed(1)) {
            this.has_note = true;
            this.timer.stop();
        }

        super.update();
    }

    /**
     * Getters and Setters
     */
    public void setHasNote(boolean has_note) { this.has_note = has_note; }
}
