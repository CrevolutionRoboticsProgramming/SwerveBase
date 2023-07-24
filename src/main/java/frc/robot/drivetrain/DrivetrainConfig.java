package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.CrevoLib.math.Conversions;

public class DrivetrainConfig {
    // Physical Constants
    public static final double kDriveGearRatio = (6.12/1.0);
    public static final double kAngleGearRatio = ((-150.0/7)/1.0);

    public static final double kDrivetrainWidth = Units.inchesToMeters(26.25);
    public static final double kDrivetrainHeight = Units.inchesToMeters(26.25);
    public static final double kDrivetrainActualLength = Units.inchesToMeters(28.0);
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelCircumfurence = (kWheelDiameter * Math.PI);

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kDrivetrainHeight / 2.0, kDrivetrainWidth / 2.0),
            new Translation2d(kDrivetrainHeight / 2.0, -kDrivetrainWidth / 2.0),
            new Translation2d(-kDrivetrainHeight / 2.0, kDrivetrainWidth / 2.0),
            new Translation2d(-kDrivetrainHeight / 2.0, -kDrivetrainWidth / 2.0));

    // Devices
    public static final SwerveModule.Config kFrontLeftModuleConfig = new SwerveModule.Config(
            1,
            3,
            2,
            Rotation2d.fromDegrees(84.63)
    );

    public static final SwerveModule.Config kFrontRightModuleConfig = new SwerveModule.Config(
            10,
            12,
            11,
            Rotation2d.fromDegrees(259.45)
    );

    public static final SwerveModule.Config kBackLeftModuleConfig = new SwerveModule.Config(
            4,
            6,
            5,
            Rotation2d.fromDegrees(17.226)
    );

    public static final SwerveModule.Config kBackRightModuleConfig = new SwerveModule.Config(
            7,
            9,
            8,
            Rotation2d.fromDegrees(166.72)
    );

//     public static final int kPigeonId = 13;
    public static final boolean kGyroInvert = true;

    // Controller Constants
    public static final double kDriveS = (0.48665 / 12.0);
    public static final double kDriveV = (2.4132 / 12.0);
    public static final double kDriveA = (0.06921 / 12.0);

    public static final double kDriveP = 0.01; // 0.08;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0; // 0.1;

    public static final double kAngleP = 0.09;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.1;

    public static final double kMaxVelocity = Units.feetToMeters(18);
    public static final double kMaxAngularVelocity = Math.PI * 4.12 * 0.4;

    public static final double kAngleAllowableClosedLoopError = Conversions.degreesToFalcon(3.0, Math.abs(kAngleGearRatio));

    /*Neutral Modes*/
    public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
    public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;

    /*Open and Closed Loop Ramping*/
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    public static CANCoderConfiguration getCANCoderConfiguration(boolean inverted) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = inverted;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }

    public static TalonFXConfiguration getDriveMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
        config.slot0.kP = kDriveP;
        config.slot0.kI = kDriveI;
        config.slot0.kD = kDriveD;
        config.slot0.kF = 0;
        config.supplyCurrLimit = limit;
        config.openloopRamp = kOpenLoopRamp;
        config.closedloopRamp = kClosedLoopRamp;
        return config;
    }

    public static TalonFXConfiguration getAngleMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
        config.slot0.kP = kAngleP;
        config.slot0.kI = kAngleI;
        config.slot0.kD = kAngleD;
        config.slot0.kF = 0;
        config.supplyCurrLimit = limit;
        return config;
    }

    public static final double kSlowModeTranslationModifier = 0.25;
    public static final double kSlowModeRotationModifier = 0.5;

    public static final double kIntakeModeTranslationModifier = 0.75;
    public static final double kIntakeModeRotationModifier = 0.6;

    public static final class SwerveDrivetrainConstants {

        /*Swerve Drivetrain Construction */
        public static final boolean FIELD_RELATIVE = true;
        public static final boolean OPEN_LOOP = false;

        //Declaring CAN ID's for each module
        //Back Left -> Mod # 4
        //Back Right -> Mod # 1
        //Front Left -> Mod # 3
        //Front Right -> Mod # 2
        
        public static final int FRONT_RIGHT_DRIVE_ID = 10;
        public static final int FRONT_RIGHT_ANGLE_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;

        
        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_ANGLE_ID = 3;
        public static final int FRONT_LEFT_CANCODER_ID = 2; 

        
        public static final int BACK_LEFT_DRIVE_ID = 4;
        public static final int BACK_LEFT_ANGLE_ID = 6;
        public static final int BACK_LEFT_CANCODER_ID = 5;

        
        public static final int BACK_RIGHT_DRIVE_ID = 7;
        public static final int BACK_RIGHT_ANGLE_ID = 9;
        public static final int BACK_RIGHT_CANCODER_ID = 8;

        //Pigeon Constants
        public static final int PIGEON_ID = 13;

        /* Module CANCoder Offsets */
        public static double FRONT_LEFT_OFFSET = 176.66; // 176.30
        public static double FRONT_RIGHT_OFFSET = 203.99; 
        public static double BACK_LEFT_OFFSET = 100.98;
        public static double BACK_RIGHT_OFFSET = 64.59;

        // public static double FRONT_LEFT_OFFSET = 176.66;
        // public static double FRONT_RIGHT_OFFSET = 207.42; 
        // public static double BACK_LEFT_OFFSET = 99.75;
        // public static double BACK_RIGHT_OFFSET = 67.93;

        /*Motor Invert Constants*/
        //Drive Motor Invert
        public static boolean FRONT_RIGHT_DRIVE_INVERT = true;
        public static boolean FRONT_LEFT_DRIVE_INVERT = true;
        public static boolean BACK_LEFT_DRIVE_INVERT = true;
        public static boolean BACK_RIGHT_DRIVE_INVERT = true;

        //Angle Motor Invert
        public static boolean FRONT_RIGHT_ANGLE_INVERT = false;
        public static boolean FRONT_LEFT_ANGLE_INVERT = false;
        public static boolean BACK_LEFT_ANGLE_INVERT = false;
        public static boolean BACK_RIGHT_ANGLE_INVERT = false;

        //CANCoder Invert
        public static boolean FRONT_RIGHT_CANCODER_INVERT = false;
        public static boolean FRONT_LEFT_CANCODER_INVERT = false;
        public static boolean BACK_LEFT_CANCODER_INVERT = false;
        public static boolean BACK_RIGHT_CANCODER_INVERT = false;

        //Pigeon Invert
        public static boolean PIGEON_INVERT = false;

        /*Drive Motor PID Values*/
        public static final double DRIVE_P = 0.08;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.1;
        public static final double DRIVE_F = 0.001;

        /*Angle Motor PID Values*/
        public static final double ANGLE_P = 0.09;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.1;
        public static final double ANGLE_F = 0.0;

        /*Drive Motor Characterization*/
        //Divide each by 12 to convert to volts for CTRE
        // public static final double DRIVE_kS = (0.19943 / 12);
        // public static final double DRIVE_kV = (0.10186 / 12);
        // public static final double DRIVE_kA = (0.016307 / 12);

        public static final double DRIVE_kS = (0.48665 / 12.0);
        public static final double DRIVE_kV = (2.4132 / 12.0);
        public static final double DRIVE_kA = (0.06921 / 12.0);

        /*Drive Motor Current Limiting*/
        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*Angle Motor Current Limiting*/
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25; 
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40; 
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        /*Neutral Modes*/
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;

        /*Swerve Gear Ratios*/
        //From SDS Website
        public static final double DRIVE_GEAR_RATIO = (6.75/1.0);
        public static final double ANGLE_GEAR_RATIO = ((-150.0/7)/1.0);
        
        /*Swerve Profiling Values*/
        public static final double MAX_SPEED = (Units.feetToMeters(18.0)); //Max from SDS Limit Speed
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5;

        /*Swerve Kinematics Constants*/
        public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(20.25);
        public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(28.75);
        public static final double DRIVETRAIN_ACTUAL_LENGTH = Units.inchesToMeters(28.0);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER  * Math.PI);

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
            new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

        /*Open and Closed Loop Ramping*/
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double ANGLE_ALLOWABLE_CL_ERROR = Conversions.degreesToFalcon(3.0, Math.abs(ANGLE_GEAR_RATIO));
    }
}
