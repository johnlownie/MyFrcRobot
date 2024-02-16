package frc.robot.controls;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * 
 */
public interface ControlBindings {
    /* Game Specific Assignments */
    Optional<Trigger> alignAndShoot();
    Optional<Trigger> driveTeleop();
    Optional<Trigger> driveToAmp();
    Optional<Trigger> driveToSpeakerRight();
    Optional<Trigger> driveToSpeakerCenter();
    Optional<Trigger> driveToSpeakerLeft();
    Optional<Trigger> resetPose();
    Optional<Trigger> reseedSteerMotors();

    DoubleSupplier omega();
    DoubleSupplier translationX();
    DoubleSupplier translationY();

    Supplier<Optional<Rotation2d>> heading();

    BooleanSupplier driverWantsControl();
    BooleanSupplier operatorWantsControl();

    /* Generic Driver Assignments */
    Optional<Trigger> driverA();
    Optional<Trigger> driverB();
    Optional<Trigger> driverX();
    Optional<Trigger> driverY();
    Optional<Trigger> driverStart();
    Optional<Trigger> driverBack();
    Optional<Trigger> driverLeftTrigger();
    Optional<Trigger> driverRightTrigger();
    Optional<Trigger> driverLeftBumper();
    Optional<Trigger> driverRightBumper();
    Optional<Trigger> driverPovLeft();
    Optional<Trigger> driverPovRight();
    Optional<Trigger> driverPovUp();
    Optional<Trigger> driverPovDown();

    /* Generic Operator Assignments */
    Optional<Trigger> operatorA();
    Optional<Trigger> operatorB();
    Optional<Trigger> operatorX();
    Optional<Trigger> operatorY();
    Optional<Trigger> operatorStart();
    Optional<Trigger> operatorBack();
    Optional<Trigger> operatorLeftTrigger();
    Optional<Trigger> operatorRightTrigger();
    Optional<Trigger> operatorLeftBumper();
    Optional<Trigger> operatorRightBumper();
    Optional<Trigger> operatorPovLeft();
    Optional<Trigger> operatorPovRight();
    Optional<Trigger> operatorPovUp();
    Optional<Trigger> operatorPovDown();
}
