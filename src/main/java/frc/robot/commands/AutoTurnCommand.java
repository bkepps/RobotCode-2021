// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrain;
import frc.robot.subsystems.MyDriveTrain;

public class AutoTurnCommand extends CommandBase {
  /** Creates a new AutoTurnCommand. */

  MyDriveTrain locDriveTrain;
  Double locRSpeed = 0.0;
  Double initialAngle = 0.0;
  Double locAngle = 0.0;        //angle in degrees

  public AutoTurnCommand(MyDriveTrain driveTrain, double rSpeed, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    locDriveTrain = driveTrain;
    locRSpeed = rSpeed;
    locAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
