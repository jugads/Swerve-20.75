package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

public class Straighten extends CommandBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.FieldCentric drive;
    SwerveRequest.FieldCentric updatedDrive;
    private final PIDController controller = new PIDController(0.025, 0., 0.004);
    private final PIDController controller1 = new PIDController(0.045, 0., 0.004);
    private final Telemetry telemetry;

    public Straighten(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
        this.m_drivetrain = drivetrain;
        this.drive = drive;
        this.telemetry = new Telemetry(kMaxSpeed);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(0.);
        controller.setTolerance(0.1);
        m_drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    @Override
    public void execute() {
        telemetry.telemeterize(m_drivetrain.getState());
        if (telemetry.getCurrentRot() <= 45) { // Ensure telemetry is up to date
        updatedDrive = drive
            .withVelocityX(0.)
            .withVelocityY(0.)
            .withRotationalRate(kMaxSpeed * controller1.calculate(telemetry.getCurrentRot()));
        }
        if (telemetry.getCurrentRot() > 45) { // Ensure telemetry is up to date
        updatedDrive = drive
            .withVelocityX(0.)
            .withVelocityY(0.)
            .withRotationalRate(kMaxSpeed * controller.calculate(telemetry.getCurrentRot()));
        }
        m_drivetrain.applyRequest(() -> updatedDrive);
    }

    @Override
    public void end(boolean interrupted) {
         // Stop drivetrain
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
