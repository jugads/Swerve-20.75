// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Telemetry.*;
public class Straighten extends Command {
  /** Creates a new Straighten. */
  CommandSwerveDrivetrain m_drivetrain;
  SwerveRequest.FieldCentric drive;
  PIDController controller = new PIDController(0.01, 0., 0.);
  private final Telemetry telemetry = new Telemetry(kMaxSpeed);


  public Straighten(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
    m_drivetrain = drivetrain;
    this.drive = drive;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0.);
    controller.setTolerance(0.1);
    m_drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.applyRequest(() -> drive.withVelocityX(0.) // Drive forward with
    // negative Y (forward)
    .withVelocityY(0.) // Drive left with negative X (left)
    .withRotationalRate(kMaxSpeed * controller.calculate(telemetry.getCurrentRot()))); // Drive counterclockwise with negative X (left)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
