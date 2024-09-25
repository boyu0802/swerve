package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.swerveDriveConstants;
import frc.robot.lib.*;


public class swerveModules {
    private int moduleNumber;
    private CANSparkMax angleMotor;
    private CANSparkFlex driveMotor;
    private CANcoder cancoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleRelativeEncoder;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private SparkPIDController drivePID;
    private SparkPIDController anglePID;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(swerveDriveConstants.kS, swerveDriveConstants.kV,swerveDriveConstants.kA);
    
    

    public swerveModules(int moduleNumber, swerveModuleConstants swerveModuleConstants ){
        this.moduleNumber = moduleNumber;
        angleMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderID);
        
    }

}
