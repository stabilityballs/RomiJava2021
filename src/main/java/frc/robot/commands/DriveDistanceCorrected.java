// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceCorrected extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_distance;
  private double pVal = 0.0885;

  /**
   * Creates a new DriveDistanceCorrected. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will turn, 1 being fastest and 0 being completely stopped
   * (0.3ish is the slowest the robot will physically go). Negative values = reverse turning
   * @param inches The number of inches the robot will drive
   * @param drive The drive subsystem on which this command will run, required so that multiple methods
   * cannot be sending conflicting values to the motors
   */
  public DriveDistanceCorrected(double speed, double inches, Drivetrain drive) {
    m_speed = speed;
    m_distance = inches;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop motors, just to be safe
    m_drive.arcadeDrive(0, 0);
    // Reset gyro so we have a point of reference for keeping robot straight
    // This means that however the robot is facing when this method is called,
    // it will attempt to keep that angle (as it thinks that is "straight"; 0)
    m_drive.resetGyro();
    // Reset encoders so we have a point of reference for distance
    m_drive.resetEncoders();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Simple variant - we think we can measure/change P value, starts 
    // at 1 for first test
    // Test and tune the P value (defined in beginning) so we can stop osciliaton
    // do so here: https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html#tuning-methods
    double driveError = -m_drive.getGyroAngleZ();
    double turnPower = pVal * driveError;
    m_drive.arcadeDrive(m_speed, turnPower);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop driving once the drive is complete
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}