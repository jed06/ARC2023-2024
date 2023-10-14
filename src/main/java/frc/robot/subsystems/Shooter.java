// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private double flywheelTolerance = 0.05; // Tolerance of PID controller
  private boolean override = false; // Helps us switch from manual to auto
  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
  private double overrideTime = 1.0;

  private ShuffleboardTab flyTab = Shuffleboard.getTab("Flywheel");
  GenericEntry nativeVel = flyTab.add("TickVelocity", 0.0).getEntry();
  GenericEntry nativePos = flyTab.add("TickPos", 0.0).getEntry();
  GenericEntry rpmEntry = flyTab.add("RPM", 0.0).getEntry();


  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.LeftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.RightFlywheelPort);

  private final PIDController leftFlywheelPID = new  PIDController(Constants.leftFlywheelPIDConsts.pidP, Constants.leftFlywheelPIDConsts.pidI, Constants.leftFlywheelPIDConsts.pidD);
  private final PIDController rightFlywheelPID = new  PIDController(Constants.rightFlywheelPIDConsts.pidP, Constants.rightFlywheelPIDConsts.pidI, Constants.rightFlywheelPIDConsts.pidD);

  private SimpleMotorFeedforward leftFlywheelFF = new  SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new   SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);


  private final LinearSystem<N1, N1, N1> flywheelPlant =
  LinearSystemId.identifyVelocitySystem(Constants.FlywheelSimConstants.kV,  Constants.FlywheelSimConstants.kA);

  private final TalonSRXSimCollection LeftFlyWheelSimMotor;
  private final FlywheelSim LeftFlyWheelSim;

  private static final double kFlywheelGearing = 1.0;

  private double motorVoltSim = 0.0;

  public Shooter() {

     // Assigns simulation and motor objects
    LeftFlyWheelSim = new FlywheelSim(flywheelPlant, DCMotor.getBag(0), kFlywheelGearing);

    LeftFlyWheelSimMotor = leftFlywheel.getSimCollection();

    leftFlywheel.configFactoryDefault();
    leftFlywheel.setInverted(true); leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(false);    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //Set the PID tolerance
    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);
    //Start and reset the timer
    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer

  } 
  
  public void setFlywheelPower(double speed) {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);

    motorVoltSim = speed *12.0;
  }
  
  public boolean flywheelWithinErrorMargin() {
    return (leftFlywheelPID.atSetpoint() && rightFlywheelPID.atSetpoint());
  }
  
  public void setFlywheelConstantVelocity(double RPM) {
    leftFlywheel.setVoltage((leftFlywheelFF.calculate(RPM/60.0)) + leftFlywheelPID.calculate(getLeftRPM(), RPM));
    rightFlywheel.setVoltage((rightFlywheelFF.calculate(RPM/60.0)) + rightFlywheelPID.calculate(getRightRPM(), RPM));
  }

  public double getLeftRPM() {
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getRightRPM() {
    return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getLeftFlywheelPower() {
    return leftFlywheel.get();
  }

  public double getRightFlywheelPower() {
    return rightFlywheel.get();
  }
    
  public double getAverageRPM() {
    return ((getLeftRPM() + getRightRPM())/2.0);
  }

  public double getFlywheelCurrent() {
    return (leftFlywheel.getStatorCurrent() + rightFlywheel.getStatorCurrent())/2.0;
  }

  public void resetFlywheelEncoders() {
    leftFlywheel.setSelectedSensorPosition(0, 0, 10);
    rightFlywheel.setSelectedSensorPosition(0, 0, 10);
  }
 
  private int velocityToNativeUnits(double velocityRPM) {
    double motorRotationsPerSecond = velocityRPM / 60.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Average Current", getFlywheelCurrent());
    SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
    SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());

    nativeVel.setDouble(leftFlywheel.getSelectedSensorVelocity());
    nativePos.setDouble(leftFlywheel.getSelectedSensorPosition());
    rpmEntry.setDouble(LeftFlyWheelSim.getAngularVelocityRPM());

    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) {
      override = !override;
      overrideTimer.reset();
      }
  }

  @Override
  public void simulationPeriodic() {
    // Set inputs of voltage
    LeftFlyWheelSim.setInput(motorVoltSim);
    // Update with dt of 0.02
    LeftFlyWheelSim.update(0.02);

    // Update Quadrature
    // Only velocity can be set - position can't be set

    LeftFlyWheelSimMotor.setQuadratureVelocity(
        velocityToNativeUnits(
            LeftFlyWheelSim.getAngularVelocityRPM()));
  }
}
