// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final TalonFX motor;
  private final AnalogInput sensor_clock_wise;
  private final AnalogInput sensor_counter_clockwise;
  /** Creates a new turret. */
  public Turret() {
    motor = new TalonFX(Constants.MOTOR_ID);
    motor.config_kP(0, 1);

    sensor_clock_wise = new AnalogInput(Constants.SENSOR_CLOCKWISE);
    sensor_counter_clockwise = new AnalogInput(Constants.SENSOR_COUNTER_CLOCKWISE);
  }

  public Pose2d getPose2d() {
    return null;
  }

  public void setPower(double power){
    motor.set(ControlMode.PercentOutput, power);  
  }

  public void setAngle(double angle) {
    motor.setSelectedSensorPosition(angle*Constants.PULSE_PER_ROTATION);
  }

  public double getAngle() {
    return motor.getSelectedSensorPosition() / Constants.PULSE_PER_ROTATION;
  }

  public double getDiffAngle(double wantedAngle) {
    double currentAngle = getAngle(), diff = wantedAngle - getAngle();
    if (diff > 180) 
      diff -= 360;
    else if (diff < -180)
      diff += 360;

    if (currentAngle + diff > Constants.MOTION_RANGE) 
      diff -= 360;
    else if (currentAngle + diff < -Constants.MOTION_RANGE)
      diff += 360;
    return diff;
  }

  public void goToAngle(double wantedAngle) {
    motor.set(ControlMode.Position, (getAngle() + getDiffAngle(wantedAngle)) * Constants.PULSE_PER_ROTATION);
  }

  public boolean getSensorClockWise(){
    return(sensor_clock_wise.getAverageVoltage()>Constants.SENSOR_ACIVATE_VALUE);
  }

  public boolean getSensorCounterClockWise(){
    return(sensor_counter_clockwise.getAverageVoltage()>Constants.SENSOR_ACIVATE_VALUE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Angle", this::getAngle, null);
  }
}
