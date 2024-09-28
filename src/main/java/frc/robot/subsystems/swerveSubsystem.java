package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveDriveConstants;
import frc.robot.swerveModules;

public class swerveSubsystem extends SubsystemBase {
    private final swerveModules frontLeftModule = new swerveModules(swerveDriveConstants.frontLeftSwerveID, swerveDriveConstants.frontLeftModule);
    private final swerveModules frontRightModule = new swerveModules(swerveDriveConstants.frontRightSwerveID, swerveDriveConstants.frontRightModule);
    private final swerveModules backLeftModule = new swerveModules(swerveDriveConstants.backLeftSwerveID, swerveDriveConstants.backLeftModule);
    private final swerveModules backRightModule = new swerveModules(swerveDriveConstants.backRightSwerveID, swerveDriveConstants.backRightModule);

    private final AHRS navx = new AHRS();

    

}
