package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final double ELEVATOR_KP = 0.0001;
  private final double ELEVATOR_KI = 0;
  private final double ELEVATOR_KD = 0;
  private final double ELEVATOR_KS = 0;
  private final double ELEVATOR_KV = 0;
  private final double ELEVATOR_KG = 0;
  private final double ELEVATOR_KA = 0;

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ELEVATOR_KP,
          ELEVATOR_KI,
          ELEVATOR_KD,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ELEVATOR_KS,
          ELEVATOR_KG,
          ELEVATOR_KV,
          ELEVATOR_KA);
  
  private TalonFX m_motor;
  private TalonFX m_motor2;
  
    // Simulation classes help us simulate what's going on, including gravity.
    /*private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            m_elevatorGearbox,
            Constants.kElevatorGearing,
            Constants.kCarriageMass,
            Constants.kElevatorDrumRadius,
            Constants.kMinElevatorHeightMeters,
            Constants.kMaxElevatorHeightMeters,
            true,
            0,
            0.01,
            0.0);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final PWMSim m_motorSim = new PWMSim(m_motor);
  
    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
  
    /** Subsystem constructor. */
    public Elevator() {
      //m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
      m_motor = new TalonFX(36);
      var talonFXConfigurator = m_motor.getConfigurator();
      var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
      limitConfigs.StatorCurrentLimit = 3;
      limitConfigs.StatorCurrentLimitEnable = true;
      limitConfigs.SupplyCurrentLimit = 3;
      limitConfigs.SupplyCurrentLimitEnable = true;
      talonFXConfigurator.apply(limitConfigs);


      m_motor2 = new TalonFX(36);
      var talonFXConfigurator2 = m_motor2.getConfigurator();

        // enable stator current limit
      talonFXConfigurator2.apply(limitConfigs);
      m_motor2.setControl(new Follower(m_motor.getDeviceID(), false));
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    //SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /** Advance the simulation. */
  /*public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_motor.getPosition().getValueAsDouble());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  /*public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }*/

  @Override
  public void close() {
    m_motor.close();
    //m_mech2d.close();
  }
}


