/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.RobotMap;
import org.usfirst.frc7280.mecanum_drive_test.commands.ClimbBack;
// import org.usfirst.frc7280.mecanum_drive_test.commands.ClimbTest;
import org.usfirst.frc7280.mecanum_drive_test.commands.ManualClimb;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public TalonSRX backClimbMotor = new TalonSRX(RobotMap.backClimbMotor);
  public TalonSRX leftMotor = new TalonSRX(RobotMap.leftMotor);
  public TalonSRX rightMotor = new TalonSRX(RobotMap.rightMotor);
  public VictorSPX climbMotionMotor = new VictorSPX(RobotMap.climbMotionMotor);

  private int level = 0;
  private int frontLevel;
  private int backLevel;

  RobotMap robotMap = new RobotMap();

  public Climb(){
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    backClimbMotor.configFactoryDefault();

    robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbPeakOutput);
    robotMap.TalonSRXInit(leftMotor, Constants.kClimbPeakOutput);
    robotMap.TalonSRXInit(rightMotor, Constants.kClimbPeakOutput);


    rightMotor.setInverted(true);
    leftMotor.setInverted(true);
    climbMotionMotor.setInverted(true);

    //rightMotor.follow(leftMotor);

    robotMap.setMotorPID(backClimbMotor, 0, 0.14, 0, 0);
    robotMap.setMotorPID(leftMotor, 0, 0.15, 0, 0);
    robotMap.setMotorPID(rightMotor, 0, 0.15, 0, 0);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    backClimbMotor.setNeutralMode(NeutralMode.Brake);

    backClimbMotor.setSensorPhase(false);
    rightMotor.setSensorPhase(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ManualClimb());
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Right leg pos", rightMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left leg pos", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Back leg pos", backClimbMotor.getSelectedSensorPosition());
  }
  
 public void climbStage(int _level){
    int step = _level / 4;

    // robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbPeakOutput);
    // robotMap.TalonSRXInit(leftMotor, Constants.kClimbPeakOutput);
    // robotMap.TalonSRXInit(rightMotor, Constants.kClimbPeakOutput);

    // backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    // leftMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    // backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);

    
    if (Math.abs(Math.abs(leftMotor.getSelectedSensorPosition()) - level)  <= 1000 
        && Math.abs(Math.abs(rightMotor.getSelectedSensorPosition()) - level) <= 1000
        && Math.abs(Math.abs(backClimbMotor.getSelectedSensorPosition()) - level) <= 1200){

      if (level >= _level){
        level = _level;
      } else {
        level += step;
      }

    } else {
      leftMotor.set(ControlMode.Position, level);
      rightMotor.set(ControlMode.Position, level);
      backClimbMotor.set(ControlMode.Position, level);
      
      System.out.println(leftMotor.getErrorDerivative());

      SmartDashboard.putNumber("clim num", level);
      SmartDashboard.putNumber("frontMaster pos", leftMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("frontSlave pos", rightMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("back pos", backClimbMotor.getSelectedSensorPosition());

    }
    frontLevel = _level;
    backLevel = _level;
    SmartDashboard.putNumber("front Level", frontLevel);
    SmartDashboard.putNumber("back Level", backLevel);

    // leftMotor.set(ControlMode.Velocity, 750);
    // rightMotor.set(ControlMode.Velocity, 750);
    // backClimbMotor.set(ControlMode.Velocity, 750);

  }

  public void retrieveBack(){
    // robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbBackOutput);

    backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);

    backClimbMotor.set(ControlMode.Position, 0);
    Robot.base.drive(-0.2, 0, 0);
  }

  public void retrieveFront() {
    // robotMap.TalonSRXInit(leftMotor, Constants.kClimbBackOutput);
    // robotMap.TalonSRXInit(rightMotor, Constants.kClimbBackOutput);

    leftMotor.configClosedLoopPeakOutput(0, Constants.kClimbBackOutput);
    backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbBackOutput);
    Robot.base.drive(-0.2, 0, 0);
    

    leftMotor.set(ControlMode.Position, 0);
    rightMotor.set(ControlMode.Position, 0);
    Robot.base.drive(-0.2, 0, 0);
  }

  public void climbMotion(){
    climbMotionMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void motionStop(){
    climbMotionMotor.set(ControlMode.PercentOutput, 0);

  }

  public void climbStop(){
    leftMotor.set(ControlMode.PercentOutput, 0);
    leftMotor.set(ControlMode.PercentOutput, 0);
    leftMotor.set(ControlMode.PercentOutput, 0);
  }



  public void climbUp(double yValue){
    double targetPosition;
    leftMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 0.8);
    rightMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 0.8);
    backClimbMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 0.8);
    // targetPosition = Robot.judge.climbLevel * Math.abs(yValue);
    targetPosition = Robot.judge.climbLevel;
    SmartDashboard.putBoolean("climb up", false);



      if(targetPosition > backClimbMotor.getSelectedSensorPosition()){
        robotMap.setMotorPID(backClimbMotor, 0, 0.2, 0, 0);
          backClimbMotor.set(ControlMode.Velocity, 3000);
      }else{
          robotMap.setMotorPID(backClimbMotor, 0, 0.005, 0, 0);
          backClimbMotor.set(ControlMode.Position, Robot.judge.climbLevel);
      }

      if( targetPosition > rightMotor.getSelectedSensorPosition()){
          robotMap.setMotorPID(leftMotor, 0, 0.2, 0, 0);
          rightMotor.set(ControlMode.Velocity, 3100);
      }else{
          robotMap.setMotorPID(leftMotor, 0, 0.005, 0, 0);
          rightMotor.set(ControlMode.Position, Robot.judge.climbLevel);
      }
        
      if( targetPosition > leftMotor.getSelectedSensorPosition()){
          robotMap.setMotorPID(rightMotor, 0, 0.2, 0, 0);
          leftMotor.set(ControlMode.Velocity, 3000);
      }else{
          robotMap.setMotorPID(rightMotor, 0, 0.008, 0, 0);
          leftMotor.set(ControlMode.Position, Robot.judge.climbLevel);
      }
    
   
    
    // targetPosition = - yValue * Robot.judge.climbLevel;
    // if(targetPosition > backClimbMotor.getSelectedSensorPosition()
    // && targetPosition > rightMotor.getSelectedSensorPosition()
    // && targetPosition > leftMotor.getSelectedSensorPosition()){
    //   backClimbMotor.set(ControlMode.Position, targetPosition);
    //   rightMotor.set(ControlMode.Position, targetPosition);
    //   leftMotor.set(ControlMode.Position, targetPosition);
    // }

      SmartDashboard.putBoolean("climb up", true);
    
    

    SmartDashboard.putNumber("climb target", targetPosition);

    SmartDashboard.putNumber("right percent", rightMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Right leg pos", rightMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left leg pos", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("back leg pos", backClimbMotor.getSelectedSensorPosition());

    SmartDashboard.putBoolean("climb", true);
    SmartDashboard.putBoolean("y>0", false);
    SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
  }


  public void climbDown(double yValue){
    double targetPosition;
    targetPosition = (- Robot.judge.climbLevel * yValue) + Robot.judge.climbLevel;
    robotMap.setMotorPID(backClimbMotor, 0, 0.15, 0, 0);
    robotMap.setMotorPID(leftMotor, 0, 0.15, 0, 0);
    robotMap.setMotorPID(rightMotor, 0, 0.1, 0, 0);
    if(500 <= backClimbMotor.getSelectedSensorPosition()
      && 500 <= rightMotor.getSelectedSensorPosition()
      && 500 <= leftMotor.getSelectedSensorPosition()){
        backClimbMotor.set(ControlMode.Velocity, -3000);
        rightMotor.set(ControlMode.Velocity, -3000);
        leftMotor.set(ControlMode.Velocity, -3000);
      }else{
        
      }
    SmartDashboard.putNumber("climb target", targetPosition);
    
    SmartDashboard.putNumber("Right leg pos", rightMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left leg pos", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("back leg pos", backClimbMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("climb", true);
    SmartDashboard.putBoolean("climb up", false);
    SmartDashboard.putBoolean("y>0", true);
    SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
  }

  public void climbMethod1(int _position){
    int targetPosition = _position;
    double rightSpeed = rightMotor.getSelectedSensorVelocity();
    if(rightMotor.getSelectedSensorPosition()> targetPosition){
      robotMap.setMotorPID(rightMotor, 0, 0.5, 0, 0);
      rightMotor.set(ControlMode.Velocity, -3700);
    } else {
      robotMap.setMotorPID(rightMotor, 0, 0.03, 0, 0);
      rightMotor.set(ControlMode.Position, targetPosition);
    }
    if(leftMotor.getSelectedSensorPosition()> targetPosition){
      robotMap.setMotorPID(leftMotor, 0.5, 0.6, 0, 0);
      leftMotor.set(ControlMode.Velocity, -5500);
    }  else {
      robotMap.setMotorPID(rightMotor, 0, 0.02, 0, 0);
      leftMotor.set(ControlMode.Position, targetPosition);
    }
    if(backClimbMotor.getSelectedSensorPosition()> targetPosition){
      robotMap.setMotorPID(backClimbMotor, 0, 0.3, 0, 0);
      backClimbMotor.set(ControlMode.Velocity, -1800);
    } else {
      robotMap.setMotorPID(backClimbMotor, 0, 0.02, 0, 0);
      backClimbMotor.set(ControlMode.Position, targetPosition);
    }

  }
}
