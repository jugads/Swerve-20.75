// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.kMaxAngularRate;
import static frc.robot.Constants.DrivetrainConstants.kMaxSpeed;

import java.util.Map;

// Imports
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Collect;
import frc.robot.commands.Straighten;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Telemetry;


public class RobotContainer {
  GenericEntry example;
  //Constants Defined
  private double MaxSpeed = kMaxSpeed; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI; // 3/4 of a rotation per second max angular velocity
  //Setting up Joystick and Drivetrain Object
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final PIDController turnController = new PIDController(0.0035, 0., 0.0005);
  private final PIDController driveController = new PIDController(0.08, 0., 0.00425);
  //Setting up PIDController, as well as both RobotRelative and FieldRelative SwerveRequests
  // PIDController controller = new PIDController(0.00725, 0.0001, 0.00013);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.09) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
      .withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.09) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
      
  // Set up brake and point SwerveRequests
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Set up Telemetry
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //Configure Bindings function, which contains all the controller bindings
  private void configureBindings() {
    //Set the default command for the drivetrain
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    // SmartDashboard.putNumber("X", logger.getCurrentX());
    // SmartDashboard.putNumber("Y", logger.getCurrentY());
    // SmartDashboard.putNumber()
    // Right Bumper creates RobotRelative Driving
    joystick.rightBumper()
    .whileTrue(
      drivetrain.applyRequest(() -> driveRobotRelative.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                        // negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate((joystick.getRightX() < 0 ? 1 : -1) * Math.pow(joystick.getRightX(), 2) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    )
    );

    //Left Trigger Straightens the Robot
    joystick.leftTrigger().whileTrue(
    new Straighten(drivetrain, drive)
    );
    joystick.x().whileTrue(
      drivetrain.applyRequest(() -> driveRobotRelative
            .withVelocityX(((-kMaxSpeed * driveController.calculate(drivetrain.getTY()))))
            .withRotationalRate(kMaxAngularRate * turnController.calculate(drivetrain.getTX()))
            ).until(() -> !drivetrain.getTV())
    );

    //A button Brakes
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    //B Button points modules to direction of Left Stick
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> driveRobotRelative
        .withVelocityX(3.)
        .withRotationalRate(example.getDouble(0.))));
    
    //Speed Limiter
        joystick.rightTrigger().whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY()*0.3 * MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
            .withVelocityY(joystick.getLeftX()*0.3 * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(joystick.getRightX() * 0.3 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    // reset the field-centric heading on left bumper press
    (joystick.leftBumper()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    //Simulation Stuff
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //Call configureBindings()
  public RobotContainer() {
    configureBindings();
    turnController.setSetpoint(0.);
    driveController.setSetpoint(0.);
    turnController.setTolerance(2);
    driveController.setTolerance(1);
    example = Shuffleboard.getTab("My Tab")
   .add("My Number", 0)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1))
   .getEntry();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
