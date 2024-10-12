package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class swerveModuleConstants{
    public int driveMotorID;
    public int angleMotorID;
    public int cancoderID;
    public Rotation2d angleOffset;
    
    public swerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset){
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}