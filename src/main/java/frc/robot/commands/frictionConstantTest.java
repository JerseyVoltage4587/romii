// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class frictionConstantTest extends CommandBase {
  private final Drivetrain m_drivetrain;
  private double motorLevel;
  private int runNumber;
  /** Creates a new frictionConstantTest. */
  public frictionConstantTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = Robot.getDrivetrain();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    motorLevel = 0.01;
    m_drivetrain.setLeftMotorLevel(motorLevel);
    m_drivetrain.setRightMotorLevel(motorLevel);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runNumber++;
    if (runNumber % 50 == 0) {
      motorLevel += 0.01;
    }
    m_drivetrain.setLeftMotorLevel(motorLevel);
    m_drivetrain.setRightMotorLevel(motorLevel);

    SmartDashboard.putNumber("LeftML", m_drivetrain.getLeftMotor());
    SmartDashboard.putNumber("RightML", m_drivetrain.getRightMotor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftMotorLevel(0);
    m_drivetrain.setRightMotorLevel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
