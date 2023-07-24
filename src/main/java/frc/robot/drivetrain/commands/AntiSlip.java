// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DrivetrainConfig.*;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.Drivetrain.*;

public class AntiSlip extends CommandBase {
  /** Creates a new AntiSlip. */

  SwerveModuleState[] swerveModuleStates;
  SwerveModuleState[] stopModuleStates;
  
  Drivetrain m_SwerveDrivetrain;
  public AntiSlip(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SwerveDrivetrain = drivetrain;
    addRequirements(m_SwerveDrivetrain);

    double minSpeed = 0.1 * SwerveDrivetrainConstants.MAX_SPEED;

    swerveModuleStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(minSpeed, Rotation2d.fromDegrees(225)),
                    new SwerveModuleState(minSpeed, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(minSpeed, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(minSpeed, Rotation2d.fromDegrees(45))
                };

    stopModuleStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveDrivetrain.setModuleStates(swerveModuleStates);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveDrivetrain.setModuleStates(stopModuleStates);
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
