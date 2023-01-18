// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SignalControl extends CommandBase {
  // private final intake intaker;
  // private final arm armer;
  /** Creates a new SignalControl. */
  public SignalControl(/*intake intake1,arm arm1*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.m_signalread);
    // intaker=intake1;
    // armer=arm1;
    // addRequirements(arm1);
    // addRequirements(intake1);

  }
  DigitalInput input = new DigitalInput(Constants.openmv_port);
  DigitalInput uptouchkey = new DigitalInput(2);
  DigitalInput downtouchkey = new DigitalInput(4);
  // Called when the command is initially scheduled.
  public boolean up=false;
  @Override
  public void initialize() {
    
  }
  public int flag=0;//default=manual
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //armer.set(-100);
    //SmartDashboard.putBoolean("DigitalValue",input.get());
    if(flag==1){//auto
      if (input.get()){//red ball 
        // intaker.release(0.8);
      }
      else{//blue ball
        // intaker.stop();
      }
    }
    if(flag==0){//manual
      double b=RobotContainer.stick.getRawAxis(2);
      double c=RobotContainer.stick.getRawAxis(3);
      if(RobotContainer.stick.getRawButtonPressed(2)){
        if(up){
          up=false;
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 1);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 1);
          Timer.delay(0.1);
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 0);
        }
        else{
          up=true;
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 1);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 1);
          Timer.delay(0.1);
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 0);
        }
      }
      if(up){//up
        // armer.up();
      }
      else{
        // armer.down();
      }
      SmartDashboard.putBoolean("up", up);
      if(b>0.1){
        // intaker.absorb(b);
        RobotContainer.stick.setRumble(RumbleType.kLeftRumble, b);
        RobotContainer.stick.setRumble(RumbleType.kRightRumble, b);
      }
      else if(c>0.1){
        // intaker.release(c);
        RobotContainer.stick.setRumble(RumbleType.kLeftRumble, c);
        RobotContainer.stick.setRumble(RumbleType.kRightRumble, c);
      }
      else{
        // intaker.stop();
        RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 0);
        RobotContainer.stick.setRumble(RumbleType.kRightRumble, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
