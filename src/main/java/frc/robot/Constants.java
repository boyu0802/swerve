package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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

    public static final double angleKp = 0.005;
    public static final double angleKi = 0.0;
    public static final double anglekd = 0.0;

    public static final double driveEncoderPositionToMeters = (1.0/6.0) * Units.inchesToMeters(4.0 * Math.PI);
    public static final double driveEncoderVelocityToMetersPerSecond = driveEncoderPositionToMeters / 60.0;
    public static final double cancoderPositionToDegrees = (1.0/20.0) * 360;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    
    public static final boolean driveMotorInverted = false;
    public static final boolean angleMotorInverted = false;

    public static final double maxSpeedMetersPerSecond = Units.feetToMeters(15.1);
    public static final double maxDegreePerSecond = 270;

    public static final double frontLeftAngleOffset = 0.007 * cancoderPositionToDegrees;
    public static final double frontRightAngleOffset = 0.63 * cancoderPositionToDegrees;
    public static final double backLeftAngleOffset = 0.258 * cancoderPositionToDegrees;
    public static final double backRightAngleOffset = 0.998 * cancoderPositionToDegrees;

    public static final int frontLeftSwerveID = 1;
    public static final int frontRightSwerveID = 2;
    public static final int backLeftSwerveID = 3;
    public static final int backRightSwerveID = 4;

    public static final swerveModuleConstants frontLeftModule = new swerveModuleConstants(5, 6, 3, frontLeftAngleOffset);
    public static final swerveModuleConstants frontRightModule = new swerveModuleConstants(3, 4,2, frontRightAngleOffset);
    public static final swerveModuleConstants backLeftModule = new swerveModuleConstants(7, 8, 4, backLeftAngleOffset);
    public static final swerveModuleConstants backRightModule = new swerveModuleConstants(1, 2, 1, backRightAngleOffset);

    public static final Translation2d frontLeftPosition = new Translation2d(chassisLength/2,chassisWidth/2);
    public static final Translation2d frontRightPosition = new Translation2d(chassisLength/2,-chassisWidth/2);
    public static final Translation2d backLeftPosition = new Translation2d(-chassisLength/2,-chassisWidth/2);
    public static final Translation2d backRightPosition = new Translation2d(-chassisLength/2,chassisWidth/2);
    
    public static final double driveXMaxAccel = 2.5;
    public static final double driveYMaxAccel = 2.5;
    public static final double driveRotMaxAccel = 135;

    public static final double driveDeadband = 0.05;
  }
}
