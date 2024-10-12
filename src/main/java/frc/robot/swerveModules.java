package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveDriveConstants;
import frc.robot.lib.*;


public class swerveModules extends SubsystemBase {
    private int moduleNumber;

    private CANSparkMax angleMotor;
    private CANSparkFlex driveMotor;

    private CANcoder cancoder;
    private RelativeEncoder driveEncoder;

    private Rotation2d angleOffset;
    private PIDController drivePID;
    private PIDController anglePID;
    
    
    

    public swerveModules(int moduleNumber, swerveModuleConstants moduleConstants ){
        this.moduleNumber = moduleNumber;
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        cancoder = new CANcoder(moduleConstants.cancoderID);
        angleOffset = moduleConstants.angleOffset;
        cancoderConfig();
        
        driveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        drivePID = new PIDController(swerveDriveConstants.driveKp, swerveDriveConstants.driveKi, swerveDriveConstants.driveKd);
        anglePID = new PIDController(swerveDriveConstants.angleKp, swerveDriveConstants.angleKi, swerveDriveConstants.anglekd);
        anglePID.enableContinuousInput(-180, 180);

        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(swerveDriveConstants.driveEncoderPositionToMeters);
        driveEncoder.setVelocityConversionFactor(swerveDriveConstants.driveEncoderVelocityToMetersPerSecond);
        angleMotor.getEncoder().setPositionConversionFactor(swerveDriveConstants.angleMotorPositionToDegrees);
        angleMotor.getEncoder().setVelocityConversionFactor(swerveDriveConstants.angleMotorPositionToDegrees/60);
        
        angleMotor.setSmartCurrentLimit(swerveDriveConstants.smartCurrentLimit);
        driveMotor.setSmartCurrentLimit(swerveDriveConstants.smartCurrentLimit);

        driveMotor.setIdleMode(swerveDriveConstants.driveIdleMode);
        angleMotor.setIdleMode(swerveDriveConstants.angleIdleMode);

        driveMotor.setInverted(swerveDriveConstants.driveMotorInverted);
        angleMotor.setInverted(swerveDriveConstants.angleMotorInverted);
        
        driveMotor.burnFlash();
        angleMotor.burnFlash();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getAnglePosition(){
        return (cancoder.getAbsolutePosition().getValueAsDouble()* 360) - angleOffset.getDegrees();
    }

    public void resetToStarting(){
        angleMotor.set(anglePID.calculate(cancoder.getAbsolutePosition().getValueAsDouble()* 360, angleOffset.getDegrees()));
    }

    public Rotation2d getRotation(){
        new Rotation2d();
        return Rotation2d.fromDegrees(getAnglePosition());
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getRotation());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getRotation());
    }

    public void cancoderConfig(){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoder.getConfigurator().apply(config);
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < swerveDriveConstants.driveDeadband) {
            stop();
            return;
          }
        state = SwerveModuleState.optimize(state, getState().angle);

        double ff = state.speedMetersPerSecond / swerveDriveConstants.maxSpeedMetersPerSecond;
        // SimpleMotorFeedforward kfeedforward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV, SWERVE_DRIVE_KA);
        // double ff = kfeedforward.calculate(state.speedMetersPerSecond, 4.5);
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double angleOutput = anglePID.calculate(getAnglePosition(), state.angle.getDegrees());
        driveMotor.set(ff + driveOutput);
        angleMotor.set(angleOutput);
    }

    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public double getDriveCurrent(){
        return driveMotor.getOutputCurrent();
    }

    public double getAngleCurrent(){
        return angleMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/Angle/Measured/Module_" + moduleNumber, getAnglePosition());
        SmartDashboard.putNumber("Swerve/Speed/Measured/Module_" + moduleNumber, getDriveVelocity());
        SmartDashboard.putNumber("Swerve/Distance/Module_" + moduleNumber, getDrivePosition());
    }
    

    




}