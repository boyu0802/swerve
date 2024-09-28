package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.lib.swerveModuleConstants;

public final class Constants {
  public static class swerveDriveConstants {
    // TODO: get real values
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kS = 0.0;
    
    public static final double driveKp = 0.0;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0;

    public static final double angleKp = 0.0;
    public static final double angleKi = 0.0;
    public static final double anglekd = 0.0;

    public static final double driveEncoderVelocityToMetersPerSecond = 0.0;
    public static final double driveEncoderPositionToMeters = 0.0;
    public static final double cancoderPositionToRadians = 360 * 180/Math.PI;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    
    public static final boolean driveMotorInverted = false;
    public static final boolean angleMotorInverted = false;

    public static final double maxSpeedMetersPerSecond = 0.0;

    
    public static final double translationDeadband = 0.0;

    public static final double frontLeftAngleOffset = 0.0;
    public static final double frontRightAngleOffset = 0.0;
    public static final double backLeftAngleOffset = 0.0;
    public static final double backRightAngleOffset = 0.0;

    public static final int frontLeftSwerveID = 1;
    public static final int frontRightSwerveID = 2;
    public static final int backLeftSwerveID = 3;
    public static final int backRightSwerveID = 4;

    public static final swerveModuleConstants frontLeftModule = new swerveModuleConstants(1, 1, 1, frontLeftAngleOffset);
    public static final swerveModuleConstants frontRightModule = new swerveModuleConstants(2, 2, 2, frontRightAngleOffset);
    public static final swerveModuleConstants backLeftModule = new swerveModuleConstants(3, 3, 3, backLeftAngleOffset);
    public static final swerveModuleConstants backRightModule = new swerveModuleConstants(4, 4, 4, backRightAngleOffset);
  }
}
