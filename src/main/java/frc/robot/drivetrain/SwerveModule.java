package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.SpectrumLib.swerve.CTREModuleState;
import frc.robot.CrevoLib.math.Conversions;

public class SwerveModule {
    public static class Config {
        public final int driveMotorId;
        public final int angleMotorId;
        public final int angleEncoderId;
        public final Rotation2d angleOffset;
        public final boolean driveInvert, angleInvert, angleEncoderInvert;

        public Config(int driveMotorId, int angleMotorId, int angleEncoderId, Rotation2d angleOffset) {
            this.driveMotorId = driveMotorId;
            this.angleMotorId = angleMotorId;
            this.angleEncoderId = angleEncoderId;
            this.angleOffset = angleOffset;
            this.driveInvert = true;
            this.angleInvert = false;
            this.angleEncoderInvert = false;
        }
    }

    private final int moduleId;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final TalonFX angleMotor, driveMotor;
    private final CANCoder angleEncoder;

    private final SimpleMotorFeedforward ffDriveController;

    public SwerveModule(int moduleId, Config config) {
        this.moduleId = moduleId;
        this.angleOffset = config.angleOffset;

        angleEncoder = new CANCoder(config.angleEncoderId);
        configAngleEncoder(config.angleEncoderInvert);

        angleMotor = new TalonFX(config.angleMotorId);
        configAngleMotor(config.angleInvert);

        driveMotor = new TalonFX(config.driveMotorId);
        configDriveMotor(config.driveInvert);

        ffDriveController = new SimpleMotorFeedforward(
                DrivetrainConfig.kDriveS,
                DrivetrainConfig.kDriveV,
                DrivetrainConfig.kDriveA
        );

        lastAngle = getState().angle;
    }

    public int getModuleId() {
        return moduleId;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConfig.kMaxVelocity;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    DrivetrainConfig.kWheelDiameter,
                    DrivetrainConfig.kDriveGearRatio
            );

            driveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    ffDriveController.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConfig.kMaxVelocity * 0.01)) ?
                lastAngle : desiredState.angle;

        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), DrivetrainConfig.kAngleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), DrivetrainConfig.kAngleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    /**
     *
     * Reset the module to the absolute position
     *
     */

    void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), DrivetrainConfig.kAngleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), DrivetrainConfig.kWheelCircumfurence, DrivetrainConfig.kDriveGearRatio),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), DrivetrainConfig.kWheelCircumfurence, DrivetrainConfig.kDriveGearRatio),
                getAngle()
        );
    }


    private void configAngleEncoder(boolean inverted) {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(DrivetrainConfig.getCANCoderConfiguration(inverted));
    }

    private void configAngleMotor(boolean inverted) {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(DrivetrainConfig.getAngleMotorConfiguration());
        angleMotor.setInverted(inverted);
        angleMotor.setNeutralMode(DrivetrainConfig.kAngleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(boolean inverted) {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(DrivetrainConfig.getDriveMotorConfiguration());
        driveMotor.setInverted(inverted);
        driveMotor.setNeutralMode(DrivetrainConfig.kDriveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }
}
