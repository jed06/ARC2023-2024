// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Importing methods and other files here

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Creating Drive train class
public class DriveTrain extends SubsystemBase {

//Creating motor variables
  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _rightDriveTalon;
  private final double ticksToMeters = (127.0/10581.0)/100.0;
  private DifferentialDrive _diffDrive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.LeftDriveTalonPort);
    _rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.RightDriveTalonPort);

  // Makes sure motors are going forward
    _leftDriveTalon.setInverted(false);
    _rightDriveTalon.setInverted(false);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _rightDriveTalon);


  }
//The method will run periodicly
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
//Getting y-axis values and translates to speed
public void tankDrive(double leftSpeed, double rightSpeed) {
  _rightDriveTalon.set(rightSpeed);
  _leftDriveTalon.set(leftSpeed);
}


public void resetEncoders(){
  _leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
  _rightDriveTalon.setSelectedSensorPosition(0, 0, 10);
}

public double getPosition(){
  // to get the position our robot is at, we should get the average of the position of the left and right wheel
  double leftPosition = _leftDriveTalon.getSelectedSensorPosition();
  double rightPosition = _leftDriveTalon.getSelectedSensorPosition();
  double avg = (leftPosition + rightPosition)/2;
  // we want to return our average value in meters
  return avg * ticksToMeters;

}

}