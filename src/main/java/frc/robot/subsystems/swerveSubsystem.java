package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveDriveConstants;
import frc.robot.swerveModules;

public class swerveSubsystem extends SubsystemBase {
    private final swerveModules frontLeftModule = new swerveModules(swerveDriveConstants.frontLeftSwerveID, swerveDriveConstants.frontLeftModule);
    private final swerveModules frontRightModule = new swerveModules(swerveDriveConstants.frontRightSwerveID, swerveDriveConstants.frontRightModule);
    private final swerveModules backLeftModule = new swerveModules(swerveDriveConstants.backLeftSwerveID, swerveDriveConstants.backLeftModule);
    private final swerveModules backRightModule = new swerveModules(swerveDriveConstants.backRightSwerveID, swerveDriveConstants.backRightModule);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(swerveDriveConstants.frontLeftPosition, swerveDriveConstants.frontRightPosition, swerveDriveConstants.backLeftPosition, swerveDriveConstants.backRightPosition);
    
    private final AHRS navx = new AHRS();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle(), getModulePositions());
    
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(-navx.getAngle(), 360));
    }

    public void resetNavx(){
        navx.reset();
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(getAngle(),getModulePositions(), pose);
    }
    
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }


    public void setModuleStates(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates( fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,-ySpeed,rotation,getAngle()) : new ChassisSpeeds(xSpeed,-ySpeed,rotation));
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,swerveDriveConstants.maxSpeedMetersPerSecond);
            frontLeftModule.setDesiredState(moduleStates[0]);
            frontRightModule.setDesiredState(moduleStates[1]);
            backLeftModule.setDesiredState(moduleStates[2]);
            backRightModule.setDesiredState(moduleStates[3]);            
    }

    public void stop(){
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }




}
