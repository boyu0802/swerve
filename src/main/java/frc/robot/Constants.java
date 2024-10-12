package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.lib.swerveModuleConstants;

public final class Constants {
  public static class swerveDriveConstants {
    // TODO: get real values
    public static final double chassisLength = 0.75;
    public static final double chassisWidth = 0.75;
   
    public static final double driveKp = 0.0009;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0;

    public static final double angleKp = 0.003;
    public static final double angleKi = 0.0;
    public static final double anglekd = 0.0;

    public static final double MAX_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_KS = 0.32/MAX_VOLTAGE;
    public static final double SWERVE_DRIVE_KV = 1.51/MAX_VOLTAGE;
    public static final double SWERVE_DRIVE_KA = 0.27/MAX_VOLTAGE;

    public static final double driveEncoderPositionToMeters = (1.0/6.0) * Units.inchesToMeters(4.0 * Math.PI);
    public static final double driveEncoderVelocityToMetersPerSecond = driveEncoderPositionToMeters / 60.0;
    public static final double angleMotorPositionToDegrees = (1.0/20.0) * 360.0 ;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    
    public static final boolean driveMotorInverted = true;
    public static final boolean angleMotorInverted = false;

    public static final double maxSpeedMetersPerSecond = Units.feetToMeters(15.1);
    public static final double maxDegreePerSecond = 4.5;

    public static final Rotation2d backRightAngleOffset = Rotation2d.fromRotations(0.007);
    public static final Rotation2d frontRightAngleOffset = Rotation2d.fromRotations(0.626);
    public static final Rotation2d frontLeftAngleOffset = Rotation2d.fromRotations(0.251);
    public static final Rotation2d backLeftAngleOffset = Rotation2d.fromRotations(0.993) ;

    public static final int backRightSwerveID = 1;
    public static final int frontRightSwerveID = 2;
    public static final int frontLeftSwerveID = 3;
    public static final int backLeftSwerveID = 4;

    public static final swerveModuleConstants frontLeftModule = new swerveModuleConstants(5, 6, 3, frontLeftAngleOffset);
    public static final swerveModuleConstants frontRightModule = new swerveModuleConstants(3, 4,2, frontRightAngleOffset);
    public static final swerveModuleConstants backLeftModule = new swerveModuleConstants(7, 8, 4, backLeftAngleOffset);
    public static final swerveModuleConstants backRightModule = new swerveModuleConstants(1, 2, 1, backRightAngleOffset);

    public static final Translation2d frontLeftPosition = new Translation2d(chassisLength/2,chassisWidth/2);
    public static final Translation2d frontRightPosition = new Translation2d(chassisLength/2,-chassisWidth/2);
    public static final Translation2d backLeftPosition = new Translation2d(-chassisLength/2,chassisWidth/2);
    public static final Translation2d backRightPosition = new Translation2d(-chassisLength/2,-chassisWidth/2);
    
    public static final double driveXMaxAccel = 2.5;
    public static final double driveYMaxAccel = 2.5;
    public static final double driveRotMaxAccel = 3;

    public static final double driveDeadband = 0.05;

    public static final int smartCurrentLimit = 35;
  }
}
