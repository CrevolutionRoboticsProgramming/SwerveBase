package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.DrivetrainConfig;

import java.util.function.DoubleSupplier;

public class DrivetrainCommands {
    public static Command drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        return drive(translationX, translationY, rotation, 1.0, 1.0, true, new Translation2d(0, 0));
    }

    public static Command driveSlowMode(DoubleSupplier translationX, DoubleSupplier translationY,
                                        DoubleSupplier rotation) {

        return drive(
                translationX,
                translationY,
                rotation,
                DrivetrainConfig.kSlowModeTranslationModifier,
                DrivetrainConfig.kSlowModeRotationModifier,
                true,
                new Translation2d(0, 0)
        );
    }

    // TODO: Legacy bug where Y and X inputs are inverted in the Translation2d
    private static Command drive(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            double translationModifier,
            double rotationModifier,
            
            boolean isFieldOriented,
            Translation2d rotationOffset) {
        return new TeleopDrive(
                RobotContainer.drivetrain,
                () -> new Translation2d(
                        translationY.getAsDouble(),
                        translationX.getAsDouble()
                ).times(DrivetrainConfig.SwerveDrivetrainConstants.MAX_SPEED).times(translationModifier),
                () -> Rotation2d.fromRadians(
                        rotation.getAsDouble()
                ).times(DrivetrainConfig.SwerveDrivetrainConstants.MAX_ANGULAR_VELOCITY).times(rotationModifier),
                isFieldOriented,
                rotationOffset
        );
    }
}
