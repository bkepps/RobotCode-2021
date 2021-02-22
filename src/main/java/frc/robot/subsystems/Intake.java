/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RunOutake;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final TalonSRX roller;
  

  public Intake() {

    roller = new TalonSRX(Constants.CANId.kIntake);

  }

  public void intakeRoller() {
    roller.set(ControlMode.PercentOutput, Constants.intake.kIntakeSpeed);
  }

  public void outtakeRoller() {
    roller.set(ControlMode.PercentOutput, Constants.intake.kOutakeSpeed);
  }

  public void stop() {
    roller.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run. why? idk
  }

/**
   * Creates a new RunOutake.
   *
   * @param runOutake TODO
   */
  public void RunOutake(RunOutake runOutake) {
    runOutake.m_subsystem = this;
    // Use addRequirements() here to declare subsystem dependencies.
    runOutake.addRequirements(this);
  }
}
