// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AimToTranslation extends CommandBase {
  private final Translation2d target;
  private final Turret turret;
  /** Creates a new AimToTranslation. */
  public AimToTranslation(Translation2d translation2d, Turret turret) {
    target = translation2d;
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d position = turret.getPose2d();
    Translation2d E = target.minus(position.getTranslation());
    Rotation2d angle = position.getRotation().minus(new Rotation2d(E.getX(), E.getY()));
    turret.goToAngle(angle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
