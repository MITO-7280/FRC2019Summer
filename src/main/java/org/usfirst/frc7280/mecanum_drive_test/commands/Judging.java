/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.commands;

import org.usfirst.frc7280.mecanum_drive_test.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Judging extends Command {
  public Judging() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.judge);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.judge.setManualMode();
    Robot.judge.detecting();
    Robot.judge.climbStageJudge();
    Robot.judge.highSpeedDrive();
    Robot.judge.hasBall();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
