/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.commands;

import org.usfirst.frc7280.mecanum_drive_test.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShootPannel extends Command {
  public ShootPannel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.intaker);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // setTimeout(0.7);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.intaker.armSolenoidUp();
    Robot.intaker.cylinderDown();
    Robot.intaker.take(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intaker.take(0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
