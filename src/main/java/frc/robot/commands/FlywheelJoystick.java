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

private final Shooter leftFlywheel;
private final Joystick joy;

  public FlywheelJoystick(Shooter leftFlywheel, Joystick joy) {
    this.leftFlywheel = leftFlywheel;
    this.joy = joy;
    addRequirements(leftFlywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    leftFlywheel.setFlywheelPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    leftFlywheel.setFlywheelPower(joy.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    leftFlywheel.setFlywheelPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
