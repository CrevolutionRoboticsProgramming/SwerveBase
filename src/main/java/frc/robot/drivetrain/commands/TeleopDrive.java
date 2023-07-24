package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drivetrain;

import java.util.function.Supplier;

public class TeleopDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final Translation2d rotationOffset;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    private final boolean isFieldRelative, isOpenLoop;

    public TeleopDrive(Drivetrain drivetrain, Supplier<Translation2d> translation,
                       Supplier<Rotation2d> rotation, boolean isFieldRelative, Translation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.rotationOffset = rotationOffset;
        this.translationSupplier = translation;
        this.rotationSupplier = rotation;
        this.isFieldRelative = isFieldRelative;
        this.isOpenLoop = false;

        addRequirements(this.drivetrain);
    }

    public TeleopDrive(Drivetrain drivetrain, Supplier<Translation2d> translation,
                       Supplier<Rotation2d> rotation, boolean isFieldRelative) {
        this(drivetrain, translation, rotation, isFieldRelative, new Translation2d(0, 0));
    }

    @Override
    public void execute() {
        final var translation = translationSupplier.get();
        final var rotation = rotationSupplier.get();

        drivetrain.drive(translation, rotation.getRadians(), isFieldRelative, isOpenLoop, rotationOffset);

        // System.out.println(translation.getX());
        // System.out.println(translation.getY());
    }

}
