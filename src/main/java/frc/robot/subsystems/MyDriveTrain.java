// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class MyDriveTrain extends SubsystemBase {
  /** Creates a new MyDriveTrain. */
  private final DifferentialDrive driveSys;
  CANSparkMax m_frontLeft;

  // NavX class thing
  AHRS ahrs;

  public MyDriveTrain() {
    m_frontLeft = new CANSparkMax(Constants.CANId.kDriveL1, MotorType.kBrushless);
    CANSparkMax m_rearLeft = new CANSparkMax(Constants.CANId.kDriveL2, MotorType.kBrushless);

    SpeedControllerGroup leftGroup = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

    CANSparkMax m_frontRight = new CANSparkMax(Constants.CANId.kDriveR1, MotorType.kBrushless);
    CANSparkMax m_rearRight = new CANSparkMax(Constants.CANId.kDriveR2, MotorType.kBrushless);

    SpeedControllerGroup rightGroup = new SpeedControllerGroup(m_frontRight, m_rearRight);

    driveSys = new DifferentialDrive(leftGroup, rightGroup);

    //try to set up connection to NavX, otherwise throw an error
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    ahrs.reset();
  }

  public void drive(double speed, double rotation){
    driveSys.arcadeDrive(speed,rotation);
  }

  public double getEncoderPosition()
  {
    return m_frontLeft.getEncoder().getPosition();
  }

  public double getHeading(){
    return ahrs.getAngle();
  }

  public void resetHeading(){
    ahrs.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
