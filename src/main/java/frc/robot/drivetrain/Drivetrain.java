package frc.robot.drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.drivetrain.DrivetrainConfig.SwerveDrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveMods;
    public Pigeon2 m_pigeonGyro;
    double pitchDerivative;
    double currentPitch = 0;
    double lastPitch = 0;
    double counter = 0;

    final double DRIVE_BANG_BANG_FWD = .06;
    final int DRIVE_BANG_BANG_SP = 10;

    BangBangController m_forward_bang_bang, m_reverse_bang_bang;

    public Drivetrain() {
        m_pigeonGyro = new Pigeon2(SwerveDrivetrainConstants.PIGEON_ID);
        m_pigeonGyro.configFactoryDefault();
        zeroGyro();

        m_swerveMods = new SwerveModule[]{
                new SwerveModule(0, DrivetrainConfig.kFrontLeftModuleConfig),
                new SwerveModule(1, DrivetrainConfig.kFrontRightModuleConfig),
                new SwerveModule(2, DrivetrainConfig.kBackLeftModuleConfig),
                new SwerveModule(3, DrivetrainConfig.kBackRightModuleConfig)
        };

        m_swerveOdometry = new SwerveDriveOdometry(DrivetrainConfig.kKinematics, getYaw(), getModulePositions());

        setOdometryForOdometryAlign();

        m_forward_bang_bang = new BangBangController();
        m_forward_bang_bang.setSetpoint(DRIVE_BANG_BANG_SP);
        m_reverse_bang_bang = new BangBangController();
        m_reverse_bang_bang.setSetpoint(-DRIVE_BANG_BANG_SP);

        // Timer.delay(1);
        // resetModules();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive(translation, rotation, fieldRelative, isOpenLoop, new Translation2d(0, 0));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
                      Translation2d centerOfRotation) {
        final ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
            );
        }

        SwerveModuleState[] swerveModuleStates = DrivetrainConfig.kKinematics.toSwerveModuleStates(
                chassisSpeeds,
                centerOfRotation
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConfig.kMaxVelocity);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleId()], isOpenLoop);
        }

        
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(DrivetrainConfig.kKinematics.toSwerveModuleStates(targetSpeeds));
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConfig.kMaxVelocity);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleId()], false);
        }
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometryAuton(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void setOdometryToOffset() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(), new Pose2d(-6.14, 1.21,
                Rotation2d.fromDegrees(0.0)));
    }

    public void setOdometryForOdometryAlign() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0.0), getModulePositions(), new Pose2d(13.56, 5.2,
                Rotation2d.fromDegrees(0.0)));
    }

    public double getPitch() {
        return m_pigeonGyro.getPitch();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveMods) {
            states[mod.getModuleId()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveMods) {
            positions[mod.getModuleId()] = mod.getPosition();
        }
        return positions;
    }

    public void stopSwerve() {
        Translation2d stop = new Translation2d(0, 0);
        drive(stop, 0, true, true);
    }

    public void zeroGyro() {
        m_pigeonGyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (DrivetrainConfig.kGyroInvert) ? Rotation2d.fromDegrees(360 - m_pigeonGyro.getYaw()) :
                Rotation2d.fromDegrees(m_pigeonGyro.getYaw());
    }

    public double getPitchDerivative() {
        return pitchDerivative;
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(m_pigeonGyro.getRoll());
    }

    public boolean isPitchDerivativeHigh() {
        return Math.abs(getPitchDerivative()) > (2 * 0.02);
    }

    public void resetModules() {
        for (final var module : m_swerveMods) {
            module.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {

        m_swerveOdometry.update(getYaw(), getModulePositions());

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.getModuleId() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.getModuleId() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.getModuleId() + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("real robot pose x", getPose().getX());
        SmartDashboard.putNumber("real robot pose y", getPose().getY());
        SmartDashboard.putNumber("real robot pose rot", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro", m_pigeonGyro.getYaw());
        SmartDashboard.putNumber("Pitch", getPitch());

        lastPitch = currentPitch;
        currentPitch = getPitch();
        pitchDerivative = lastPitch - currentPitch;

        SmartDashboard.putNumber("pitch rate of change", pitchDerivative);
        SmartDashboard.putBoolean("is pitch derivative high", isPitchDerivativeHigh());


    }
}
