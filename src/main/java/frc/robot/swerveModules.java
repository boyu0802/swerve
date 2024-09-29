package frc.robot;

import javax.lang.model.element.ModuleElement;

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

    private double angleOffset;
    private PIDController drivePID;
    private PIDController anglePID;
    
    
    

    public swerveModules(int moduleNumber, swerveModuleConstants moduleConstants ){
        this.moduleNumber = moduleNumber;
        angleMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        cancoder = new CANcoder(moduleConstants.cancoderID);
        angleOffset = moduleConstants.angleOffset;
        cancoderConfig();
        

        drivePID = new PIDController(swerveDriveConstants.driveKp, swerveDriveConstants.driveKi, swerveDriveConstants.driveKd);
        anglePID = new PIDController(swerveDriveConstants.angleKp, swerveDriveConstants.angleKi, swerveDriveConstants.anglekd);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(swerveDriveConstants.driveEncoderPositionToMeters);
        driveEncoder.setVelocityConversionFactor(swerveDriveConstants.driveEncoderVelocityToMetersPerSecond);
        

        driveMotor.setIdleMode(swerveDriveConstants.driveIdleMode);
        angleMotor.setIdleMode(swerveDriveConstants.angleIdleMode);

        driveMotor.setInverted(swerveDriveConstants.driveMotorInverted);
        angleMotor.setInverted(swerveDriveConstants.angleMotorInverted);

        
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getAnglePosition(){
        return (cancoder.getAbsolutePosition().getValueAsDouble()* swerveDriveConstants.cancoderPositionToRadians) - angleOffset;
    }

    public Rotation2d getRotation(){
        return new Rotation2d(getAnglePosition());
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
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double angleOutput = anglePID.calculate(getAnglePosition(), state.angle.getRadians());
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
