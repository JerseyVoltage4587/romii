// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoball extends SequentialCommandGroup {
  /** Creates a new twoball. */
  public twoball(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    Drivetrain m_drivetrain = drivetrain;
    ForwardPID commandI = new ForwardPID(8, 1, m_drivetrain, true, 0);
    printdone commandII = new printdone("don1");
    ForwardPID commandIII = new ForwardPID(4, 1, m_drivetrain, false, 0); 
    printdone commandIV = new printdone("don2");
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ForwardPID(8, 1, m_drivetrain, true, 0), new ForwardPID(4, 1, m_drivetrain, false, 0));
  }
}
