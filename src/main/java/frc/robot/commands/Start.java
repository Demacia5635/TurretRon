// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class Start extends CommandBase {
  private Turret turret;
  private boolean search;
  /** Creates a new Start. */
  public Start(Turret turret) {
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   search = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (search) {
     turret.setPower(0.1);
     
     if(turret.getSensorClockWise()) {
       turret.setAngle(160);
       search = false;
     } else if(turret.getSensorCounterClockWise()) {
       turret.setAngle(-160);
       search = false;
     }
    turret.goToAngle(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getAngle()) <= 0.3 && !search;
  }
}
