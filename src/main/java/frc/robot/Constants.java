package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.swerveModuleConstants;

public final class Constants {
  public static class swerveDriveConstants {
    // TODO: get real values
    public static final double chassisLength = 0.0;
    public static final double chassisWidth = 0.0;

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
    public static final double maxRadiansPerSecond = 0.0;

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

    public static final Translation2d frontLeftPosition = new Translation2d(chassisLength/2,chassisWidth/2);
    public static final Translation2d frontRightPosition = new Translation2d(chassisLength/2,-chassisWidth/2);
    public static final Translation2d backLeftPosition = new Translation2d(-chassisLength/2,-chassisWidth/2);
    public static final Translation2d backRightPosition = new Translation2d(-chassisLength/2,chassisWidth/2);
    
    public static final double driveXMaxAccel = 0.0;
    public static final double driveYMaxAccel = 0.0;
    public static final double driveRotMaxAccel = 0.0;

    public static final double driveDeadband = 0.0;
  }
}
