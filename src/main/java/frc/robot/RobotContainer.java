// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.kMaxSpeed;

// Imports
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Straighten;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Telemetry;


public class RobotContainer {

  //Constants Defined
  private double MaxSpeed = kMaxSpeed; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = Math.PI * 3;/// (20.75 * Math.PI * 0.0254); // 3/4 of a rotation per second max angular velocity

  //Setting up Joystick and Drivetrain Object
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  //Setting up PIDController, as well as both RobotRelative and FieldRelative SwerveRequests
  PIDController controller = new PIDController(0.00725, 0.0001, 0.00013);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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

    // Right Bumper creates RobotRelative Driving
    joystick.rightBumper()
    .whileTrue(
      drivetrain.applyRequest(() -> driveRobotRelative.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                        // negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    )
    );

    //Left Trigger Straightens the Robot
    joystick.leftTrigger().whileTrue(
      drivetrain.applyRequest(() -> driveRobotRelative.withVelocityX(0.) // Drive forward with                                                                                // negative Y (forward)
        .withVelocityY(0.) // Drive left with negative X (left)
          .withRotationalRate(controller.calculate(logger.getCurrentRot(), 0) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    )
    );

    //A button Brakes
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    //B Button points modules to direction of Left Stick
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    
    //Speed Limiter
        joystick.rightTrigger().whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY()*0.5 * MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
            .withVelocityY(joystick.getLeftX()*0.5 * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    //Simulation Stuff
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //Call configureBindings()
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
