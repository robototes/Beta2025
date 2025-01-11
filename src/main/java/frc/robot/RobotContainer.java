// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeGroundIntake;
import frc.robot.subsystems.AlgaePlacer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralPlacer;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController codriveController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AlgaePlacer algaePlacer = new AlgaePlacer();
    public final AlgaeGroundIntake algaeGroundIntake = new AlgaeGroundIntake();
    public final CoralPlacer coralPlacer = new CoralPlacer();
    private boolean slowMode = false;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * (slowMode ? 0.5 : 1)) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed * (slowMode ? 0.5 : 1)) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate * (slowMode ? 0.5 : 1)) // Drive counterclockwise with negative X (left)
            )
        );

        // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        codriveController.leftBumper().whileTrue(algaeGroundIntake.runAtSpeed(() -> codriveController.getLeftY()));
        codriveController.rightBumper().whileTrue(algaePlacer.SpinSpeed(() -> -codriveController.getRightY()));
        codriveController.y().whileTrue(coralPlacer.runAtSpeed(0.5));
        codriveController.a().whileTrue(coralPlacer.runAtSpeed(-0.05));
        codriveController.b().whileTrue(
            Commands.parallel(
                algaeGroundIntake.runAtSpeed(() -> -0.20),
                algaePlacer.SpinSpeed(() -> 0.28)));

        driveController.b().whileTrue(coralPlacer.runAtSpeed(0.5));
        driveController.a().whileTrue(coralPlacer.runAtSpeed(-0.05));
        driveController.leftBumper().whileTrue(Commands.startEnd(() -> { slowMode = true; }, () -> { slowMode = false; }));
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
