/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.commands;

import org.usfirst.frc7280.mecanum_drive_test.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbStage extends Command {

  private int level;
  private boolean finished;

  public ClimbStage(int _level) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climb);
    level = _level;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    finished = true;
    if (Robot.oi.climbStick.getPOV() == 0){
      finished = false;
    }
    // Robot.climb.climbStage(level);
    // Robot.climb.manualClimbMotion(-Robot.oi.climbStick.getRawAxis(1));
    Robot.climb.climbMethod1(Robot.judge.climbLevel);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Robot.climb.climbStage(level);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
