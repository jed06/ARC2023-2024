// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;

public class FlywheelJoystick extends CommandBase {
  /** Creates a new FlywheelJoystick. */

private final Shooter shooter;
private final Joystick joy;

  public FlywheelJoystick(Shooter shooter, Joystick joy) {
    this.shooter = shooter;
    this.joy = joy;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    shooter.setFlywheelPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.setFlywheelPower(-joy.getRawAxis(5));
    shooter.setRollerPower(-joy.getRawAxis(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
