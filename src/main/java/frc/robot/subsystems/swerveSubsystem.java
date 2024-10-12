package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveDriveConstants;
import frc.robot.swerveModules;

public class swerveSubsystem extends SubsystemBase {
    private final swerveModules frontLeftModule;
    private final swerveModules frontRightModule;
    private final swerveModules backLeftModule;
    private final swerveModules backRightModule;
    private final SwerveDriveOdometry odometry;
    private final SwerveDriveKinematics kinematics;
    private final AHRS navx;


    public swerveSubsystem(){
        navx = new AHRS();
        resetNavx();

        frontLeftModule = new swerveModules(swerveDriveConstants.frontLeftSwerveID, swerveDriveConstants.frontLeftModule);
        frontRightModule = new swerveModules(swerveDriveConstants.frontRightSwerveID, swerveDriveConstants.frontRightModule);
        backLeftModule = new swerveModules(swerveDriveConstants.backLeftSwerveID, swerveDriveConstants.backLeftModule);
        backRightModule = new swerveModules(swerveDriveConstants.backRightSwerveID, swerveDriveConstants.backRightModule);

        kinematics = new SwerveDriveKinematics(swerveDriveConstants.frontLeftPosition, swerveDriveConstants.frontRightPosition, swerveDriveConstants.backLeftPosition, swerveDriveConstants.backRightPosition);


        odometry = new SwerveDriveOdometry(kinematics, getAngle(), getModulePositions());

        Timer.delay(0.69);
        resetModulesToStarting();

    }
    

    
    
    

    
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

    public void resetModulesToStarting(){
        frontLeftModule.resetToStarting();
        frontRightModule.resetToStarting();
        backLeftModule.resetToStarting();
        backRightModule.resetToStarting();       
    }

}
