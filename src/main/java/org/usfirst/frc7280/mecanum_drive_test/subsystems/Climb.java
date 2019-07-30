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
import org.usfirst.frc7280.mecanum_drive_test.commands.ClimbTest;
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
  public TalonSRX frontMasterMotor = new TalonSRX(RobotMap.frontMasterMotor);
  public TalonSRX frontSlaveMotor = new TalonSRX(RobotMap.frontSlaveMotor);
  public VictorSPX climbMotionMotor = new VictorSPX(RobotMap.climbMotionMotor);

  private int level = 0;
  private int frontLevel;
  private int backLevel;

  RobotMap robotMap = new RobotMap();

  public Climb(){
    frontMasterMotor.configFactoryDefault();
    frontSlaveMotor.configFactoryDefault();
    backClimbMotor.configFactoryDefault();

    robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbPeakOutput);
    robotMap.TalonSRXInit(frontMasterMotor, Constants.kClimbPeakOutput);
    robotMap.TalonSRXInit(frontSlaveMotor, Constants.kClimbPeakOutput);


    frontSlaveMotor.setInverted(false);
    frontMasterMotor.setInverted(true);
    climbMotionMotor.setInverted(true);

    //frontSlaveMotor.follow(frontMasterMotor);

    robotMap.setMotorPID(backClimbMotor, 0, 0.14, 0, 0);
    robotMap.setMotorPID(frontMasterMotor, 0, 0.15, 0, 0);
    robotMap.setMotorPID(frontSlaveMotor, 0, 0.15, 0, 0);

    frontMasterMotor.setNeutralMode(NeutralMode.Brake);
    frontSlaveMotor.setNeutralMode(NeutralMode.Brake);
    backClimbMotor.setNeutralMode(NeutralMode.Brake);

    backClimbMotor.setSensorPhase(false);
    frontSlaveMotor.setSensorPhase(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ManualClimb());
  }

  public void climbStage(int _level){
    int step = _level / 4;

    frontMasterMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    frontSlaveMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);

    // robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbPeakOutput);
    // robotMap.TalonSRXInit(frontMasterMotor, Constants.kClimbPeakOutput);
    // robotMap.TalonSRXInit(frontSlaveMotor, Constants.kClimbPeakOutput);

    // backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    // frontMasterMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);
    // backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbPeakOutput);

    
    if (Math.abs(Math.abs(frontMasterMotor.getSelectedSensorPosition()) - level)  <= 1000 
        && Math.abs(Math.abs(frontSlaveMotor.getSelectedSensorPosition()) - level) <= 1000
        && Math.abs(Math.abs(backClimbMotor.getSelectedSensorPosition()) - level) <= 1200){


      if (level >= _level){
        level = _level;
      } else {
        level += step;
      }

    } else {
      frontMasterMotor.set(ControlMode.Position, level);
      frontSlaveMotor.set(ControlMode.Position, - level);
      backClimbMotor.set(ControlMode.Position, level);
      
      System.out.println(frontMasterMotor.getErrorDerivative());

      SmartDashboard.putNumber("clim num", level);
      SmartDashboard.putNumber("frontMaster pos", frontMasterMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("frontSlave pos", frontSlaveMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("back pos", backClimbMotor.getSelectedSensorPosition());

    }
    frontLevel = _level;
    backLevel = _level;
    SmartDashboard.putNumber("front Level", frontLevel);
    SmartDashboard.putNumber("back Level", backLevel);

    // frontMasterMotor.set(ControlMode.Velocity, 750);
    // frontSlaveMotor.set(ControlMode.Velocity, 750);
    // backClimbMotor.set(ControlMode.Velocity, 750);

  }

  public void retrieveBack(){
    // robotMap.TalonSRXInit(backClimbMotor, Constants.kClimbBackOutput);

    backClimbMotor.configClosedLoopPeakOutput(0, Constants.kClimbBackOutput);

    backClimbMotor.set(ControlMode.Position, 0);
    Robot.base.drive(-0.2, 0, 0);
  }

  public void retrieveFront() {
    // robotMap.TalonSRXInit(frontMasterMotor, Constants.kClimbBackOutput);
    // robotMap.TalonSRXInit(frontSlaveMotor, Constants.kClimbBackOutput);

    frontMasterMotor.configClosedLoopPeakOutput(0, Constants.kClimbBackOutput);
    frontSlaveMotor.configClosedLoopPeakOutput(0, Constants.kClimbBackOutput);
    Robot.base.drive(-0.2, 0, 0);
    

    frontMasterMotor.set(ControlMode.Position, 0);
    frontSlaveMotor.set(ControlMode.Position, 0);
    Robot.base.drive(-0.2, 0, 0);
  }

  public void climbMotion(){
    climbMotionMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void motionStop(){
    climbMotionMotor.set(ControlMode.PercentOutput, 0);

  }

  public void climbStop(){
    frontMasterMotor.set(ControlMode.PercentOutput, 0);
    frontSlaveMotor.set(ControlMode.PercentOutput, 0);
    backClimbMotor.set(ControlMode.PercentOutput, 0);
  }

  public void climbTest(double d){
    
    frontMasterMotor.set(ControlMode.Position, d);
    frontSlaveMotor.set(ControlMode.Position, -d);
    backClimbMotor.set(ControlMode.Position, d);

    SmartDashboard.putNumber("frontMaster pos", frontMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("frontSlave pos", frontSlaveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("back pos", backClimbMotor.getSelectedSensorPosition());
  }

  public void manualPutBackLeft(){
    frontMasterMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void manualPutLeft(){
    frontMasterMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void manualClimbMotion(double d){
    climbMotionMotor.set(ControlMode.PercentOutput, -d);
  }

  public void manualPutBackRight(){
    frontSlaveMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void manualPutRight(){
    frontSlaveMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void manualPutBackRear(){
    backClimbMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void manualPutRear(){
    backClimbMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void rightMotorStop(){
    frontSlaveMotor.set(ControlMode.PercentOutput, 0);
  }

  public void leftMotorStop(){
    frontMasterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void backMotorStop(){
    backClimbMotor.set(ControlMode.PercentOutput, 0);
  }

  public void manualClimb (double yValue){
    double targetPosition; // = percent y value * climbLevel(first or second)


    boolean climbFinished = false;

    if (Robot.oi.climbStick.getRawButton(4)){
      climbMotion();
    }

    if (Robot.judge.climbOk){


        if(yValue < 0){
          targetPosition = Robot.judge.climbLevel * Math.abs(yValue);
          SmartDashboard.putBoolean("climb up", false);

          if(targetPosition > backClimbMotor.getSelectedSensorPosition()
          && targetPosition > -frontSlaveMotor.getSelectedSensorPosition()
          && targetPosition > frontMasterMotor.getSelectedSensorPosition()){
            backClimbMotor.set(ControlMode.Position, targetPosition);
            frontSlaveMotor.set(ControlMode.Position, -targetPosition);
            frontMasterMotor.set(ControlMode.Position, targetPosition);
            
            SmartDashboard.putBoolean("climb up", true);
            
         
          }
          SmartDashboard.putNumber("climb target", targetPosition);

            SmartDashboard.putBoolean("climb", true);
          SmartDashboard.putBoolean("y>0", false);
          SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
        } else if (yValue > 0){
          targetPosition = Robot.judge.climbLevel * (0 - yValue) + Robot.judge.climbLevel;

          if(targetPosition < backClimbMotor.getSelectedSensorPosition()
          && targetPosition < -frontSlaveMotor.getSelectedSensorPosition()
          && targetPosition < frontMasterMotor.getSelectedSensorPosition()){
          frontMasterMotor.set(ControlMode.Position, targetPosition);
          frontSlaveMotor.set(ControlMode.Position, -targetPosition);
          backClimbMotor.set(ControlMode.Position, targetPosition);

          SmartDashboard.putBoolean("climb up", false);

          }
          SmartDashboard.putNumber("climb target", targetPosition);
          SmartDashboard.putBoolean("climb", true);
          SmartDashboard.putBoolean("climb up", false);
          SmartDashboard.putBoolean("y>0", true);
          SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
        }
      }


      //  else {
      //   if(Robot.oi.climbStick.getRawButton(2)){
      //     retrieveFront();
      // }
  // }
  
  }

  public void climbUp(double yValue){
    double targetPosition;
    targetPosition = Robot.judge.climbLevel * Math.abs(yValue);
          SmartDashboard.putBoolean("climb up", false);

          if(targetPosition > backClimbMotor.getSelectedSensorPosition()
          && targetPosition > -frontSlaveMotor.getSelectedSensorPosition()
          && targetPosition > frontMasterMotor.getSelectedSensorPosition()){
            backClimbMotor.set(ControlMode.Position, targetPosition);
            frontSlaveMotor.set(ControlMode.Position, -targetPosition);
            frontMasterMotor.set(ControlMode.Position, targetPosition);
            
            SmartDashboard.putBoolean("climb up", true);
            
         
          }
          SmartDashboard.putNumber("climb target", targetPosition);

            SmartDashboard.putBoolean("climb", true);
          SmartDashboard.putBoolean("y>0", false);
          SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
  }

  public void climbDown(double yValue){
    double targetPosition;
    targetPosition = (- Robot.judge.climbLevel * yValue) + Robot.judge.climbLevel;

    if(targetPosition < backClimbMotor.getSelectedSensorPosition()
    && targetPosition < -frontSlaveMotor.getSelectedSensorPosition()
    && targetPosition < frontMasterMotor.getSelectedSensorPosition()){
    frontMasterMotor.set(ControlMode.Position, targetPosition);
    frontSlaveMotor.set(ControlMode.Position, -targetPosition);
    backClimbMotor.set(ControlMode.Position, targetPosition);

    SmartDashboard.putBoolean("climb up", false);

    }
    SmartDashboard.putNumber("climb target", targetPosition);
    SmartDashboard.putBoolean("climb", true);
    SmartDashboard.putBoolean("climb up", false);
    SmartDashboard.putBoolean("y>0", true);
    SmartDashboard.putNumber("climb distance", targetPosition - backClimbMotor.getSelectedSensorPosition());
  }
}
