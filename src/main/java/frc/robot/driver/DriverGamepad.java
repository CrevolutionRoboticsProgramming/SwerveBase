package frc.robot.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.commands.DrivetrainCommands;

public class DriverGamepad extends Gamepad {
    public DriverGamepad() {
        super("DriverController", DriverConfig.kDriverPort);
        
        gamepad.leftStick.setDeadband(0.1);
        gamepad.rightStick.setDeadband(0.1);

        gamepad.rightStick.configCurves(2, 1);
    }

    @Override
    public void setupTeleopButtons() {
        
        gamepad.leftBumper.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.zeroGyro()));
        gamepad.selectButton.onTrue(new InstantCommand(() -> RobotContainer.drivetrain.resetModules()));

        // gamepad.rightBumper.whileTrue(new InstantCommand(() -> {while(true)System.out.println("SOMETHING");}));

        gamepad.rightBumper.whileTrue(DrivetrainCommands.driveSlowMode(
                this::getDriveTranslationX,
                this::getDriveTranslationY,
                this::getDriveRotation
        ));
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }

    public Trigger noShift() {
        return gamepad.leftTriggerButton.negate();
    }

    public Trigger shift() {
        return gamepad.leftTriggerButton;
    }

    public double getRightTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kRightTrigger.value);
    }

    public double getLeftTriggerRaw() {
        return gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }

    public double getDriveTranslationX() {
        return gamepad.leftStick.getX();
    }

    public double getDriveTranslationY() {
        return gamepad.leftStick.getY();
    }

    public double getDriveRotation() {
        return gamepad.rightStick.getX();
    }
}
