package frc.robot.modules.drawer;

import frc.lib.util.Timer;
import frc.robot.subsystems.PneumaticSubsystem;

/**
 * 
 */
public class DrawerModuleSimulator extends DrawerModule {
    private final Timer timer = new Timer();
    
    /**
     * 
     */
    public DrawerModuleSimulator(PneumaticSubsystem pneumaticSubsystem) {
        super(pneumaticSubsystem);
    }

    @Override
    public boolean hasGamePiece() {
        boolean has_game_piece = false;

        if (!this.timer.isRunning()) {
            this.timer.reset();
            this.timer.start();
        }

        if (this.timer.hasElapsed(3.0)) {
            this.timer.stop();
            has_game_piece = true;
        }

        return has_game_piece;
    }
}
