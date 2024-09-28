package frc.robot.lib;


public class swerveModuleConstants{
    public int driveMotorID;
    public int angleMotorID;
    public int cancoderID;
    public double angleOffset;
    
    public swerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, double angleOffset){
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}