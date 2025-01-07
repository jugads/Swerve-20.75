// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import java.util.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Collect extends Command {
  /** Creates a new Collect. */
  private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric drive;
    SwerveRequest.RobotCentric updatedDrive;
    private final PIDController turnController = new PIDController(0.0035, 0., 0.0005);
    private final PIDController driveController = new PIDController(0.06, 0., 0.);
    private final Telemetry telemetry;

    public Collect(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
        this.m_drivetrain = drivetrain;
        this.drive = drive;
        this.telemetry = new Telemetry(kMaxSpeed);

        addRequirements(drivetrain);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setSetpoint(0.);
    driveController.setSetpoint(-17.);
    turnController.setTolerance(2);
    m_drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.println("Command");
    // SmartDashboard.putNumber("Applied Velocity", velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
