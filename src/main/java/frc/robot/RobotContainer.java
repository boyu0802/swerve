// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerveDrive;
import frc.robot.subsystems.swerveSubsystem;



public class RobotContainer {
  private final swerveSubsystem swerve = new swerveSubsystem();
  private final XboxController controller = new XboxController(0);
  

  

  public RobotContainer() {
    swerve.setDefaultCommand(new swerveDrive(swerve,
        () ->controller.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> -controller.getRawAxis(XboxController.Axis.kLeftX.value),
        () -> controller.getRawAxis(XboxController.Axis.kRightX.value),
        () ->controller.getRightBumper()
    ));
    configureBindings();
  }


  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value) // Right Bumper
            .onTrue(new InstantCommand(swerve::resetNavx));
   
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}
