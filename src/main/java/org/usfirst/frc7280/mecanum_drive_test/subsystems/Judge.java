/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.subsystems;

import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.commands.Judging;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
public class Judge extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public boolean manualModeOn = false;
  public boolean hasBall = false;
  public boolean atButtom = true;
  public int visionDistence = 1000;
  private DigitalInput calibrateSensor = new DigitalInput(5);
  public int climbLevel;
  public boolean climbOk;
  public boolean highSpeedOn = true;
  public boolean isCalibrated = false;



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Judging());
  }

  public void setManualMode(){
    if (Robot.oi.functionStick.getPOV() == 0){
      manualModeOn = true;
    } else if (Robot.oi.functionStick.getPOV() == 180){
      manualModeOn = false;
    }
    SmartDashboard.putBoolean("Manual Mode", manualModeOn);
  }

  public void detecting(){
    isCalibrated =! calibrateSensor.get();
    SmartDashboard.putBoolean("isCalibrated", isCalibrated);
    if (isCalibrated){
      Robot.elevator.elevatorMaster.setSelectedSensorPosition(1040);
    }
  }

  public void climbStageJudge(){
    if (Robot.oi.climbStick.getRawButton(7)){
      climbLevel = Constants.kClimbFirstLevel;
      climbOk = true;
    } else if(Robot.oi.climbStick.getRawButton(8)){
      climbLevel = Constants.kClimbSecondLevel;
      climbOk = true;
    }

    SmartDashboard.putBoolean("climbok", climbOk);
  }

  public void highSpeedDrive(){
    if (Robot.oi.motionStick.getRawButton(12)){
      highSpeedOn = false;
    }
    if(Robot.oi.motionStick.getRawButton(11)){
      highSpeedOn = true;
    }

    SmartDashboard.putBoolean("high speed", highSpeedOn);
  }

  public void hasBall(){
    if (Robot.oi.motionStick.getRawButton(5)){
      hasBall = true;
    }
    if (Robot.oi.motionStick.getRawButton(6)){
      hasBall = false;
    }
    if(Robot.oi.motionStick.getRawButton(7)){
      hasBall = false;
    }
    if(Robot.oi.motionStick.getRawButton(8)){
      hasBall = false;
    }
  }
}
