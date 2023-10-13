// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DistanceAuto extends CommandBase {
  /** Creates a new DistanceAuto. */
private final DriveTrain _driveTrain;
private double position;
  public DistanceAuto(DriveTrain dt, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    this.position = position;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_driveTrain.getPosition()< position){
      _driveTrain.tankDrive(0.8, 0.8);
    }
    else{
      _driveTrain.tankDrive(0, 0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_driveTrain.getPosition() < position){
    return false;
    }
    return true;
    
  }
}
