package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.swerveDriveConstants;
import frc.robot.subsystems.swerveSubsystem;

public class swerveDrive extends Command {

    private final swerveSubsystem swerve;
    private final Supplier<Double> xSupplier, ySupplier, rotSupplier;
    private final Supplier<Boolean> fieldRelativeSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public swerveDrive(swerveSubsystem swerve, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rotSupplier, Supplier<Boolean> fieldRelativeSupplier){
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        xLimiter = new SlewRateLimiter(swerveDriveConstants.driveXMaxAccel);
        yLimiter = new SlewRateLimiter(swerveDriveConstants.driveYMaxAccel);
        rotLimiter = new SlewRateLimiter(swerveDriveConstants.driveRotMaxAccel);
        addRequirements(swerve);
    }
    
    @Override
    public void execute(){
        double xSpeed = xSupplier.get();
        double ySpeed = ySupplier.get();
        double rot = rotSupplier.get();
        boolean fieldRelative = fieldRelativeSupplier.get();

        xSpeed = Math.abs(xSpeed) > swerveDriveConstants.driveDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > swerveDriveConstants.driveDeadband ? ySpeed : 0;
        rot = Math.abs(rot) > swerveDriveConstants.driveDeadband ? rot : 0;

        xSpeed = xLimiter.calculate(xSpeed) * swerveDriveConstants.maxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * swerveDriveConstants.maxSpeedMetersPerSecond;
        rot = rotLimiter.calculate(rot) * swerveDriveConstants.maxDegreePerSecond;


        swerve.setModuleStates(xSpeed, ySpeed, rot, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
