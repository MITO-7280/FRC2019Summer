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

public class ManualClimb extends Command {

  public ManualClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climb);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("yValue", Robot.oi.climbStick.getY());
    if(Robot.judge.climbOk){
      if(Robot.oi.climbStick.getY() < 0){
      Robot.climb.climbUp(Robot.oi.climbStick.getY());
      }else {
        SmartDashboard.putBoolean("y>0", true);
        Robot.climb.climbDown(Robot.oi.climbStick.getY());
      }
      } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

        return false;


    
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

  }
}
