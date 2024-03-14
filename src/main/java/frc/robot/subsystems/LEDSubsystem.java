package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.blinkin.BlinkinController;
import frc.lib.blinkin.BlinkinPreset;
import frc.lib.led.LEDController;

public class LEDSubsystem extends SubsystemBase {
    /* Subsystems */
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    /**
     * 
     */
    public LEDSubsystem(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void periodic() {
        // 
        if (this.shooterSubsystem.hasNote()) {
            if (this.armSubsystem.isAtAngle()) {
                BlinkinController.set(BlinkinPreset.Solid.kGreen);
                LEDController.set(LEDController.Effect.SOLID, Color.kGreen);
            }
            else if (this.armSubsystem.getCurrentState() == ArmSubsystem.Action.MOVE_TO_AMP ||
                this.armSubsystem.getCurrentState() == ArmSubsystem.Action.MOVE_TO_SPEAKER) {
                    BlinkinController.set(BlinkinPreset.Strobe.kBlue);
                    LEDController.set(LEDController.Effect.STROBE, Color.kBlue);
                }
            else {
                BlinkinController.set(BlinkinPreset.Solid.kOrange);
                LEDController.set(LEDController.Effect.SOLID, Color.kOrange);
            }
        }
        else if (this.intakeSubsystem.getCurrentState() == IntakeSubsystem.Action.INTAKE ||
            this.shooterSubsystem.getCurrentState() == ShooterSubsystem.Action.INTAKE) {
                BlinkinController.set(BlinkinPreset.Strobe.kWhite);
                LEDController.set(LEDController.Effect.STROBE, Color.kWhite);
            }
        else {
            BlinkinController.set(BlinkinPreset.Solid.kBlack);
            LEDController.set(LEDController.Effect.SOLID, Color.kBlack);
        }
    }
}
