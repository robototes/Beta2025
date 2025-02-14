package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class AlgaeGroundIntake extends SubsystemBase {
  private final TalonFX motor;

  public AlgaeGroundIntake() {
    motor = new TalonFX(31);
  }

  public Command runAtSpeed(DoubleSupplier speed) {
    return run(() -> motor.set(speed.getAsDouble()))
        .finallyDo(() -> motor.set(0))
        .withName("Run at speed");
  }
}
