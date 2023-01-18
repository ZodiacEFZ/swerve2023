// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Swerve_subsystem;

public class Swerve_drive_joystick extends CommandBase {
  /** Creates a new Swerve_drive. */
  private final Swerve_subsystem swerve_subsystem;
  public boolean field_oriented=false,flag=false;
  public double targetangle=0;
  public double[] angleGoal = new double[8] , velocityGoal = new double[8];

  public Swerve_drive_joystick(Swerve_subsystem s1) {
    swerve_subsystem=s1;
    addRequirements(s1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    field_oriented=false;
  }

  public void stop_all(){
    RobotContainer.LeftBackSwerveModule.setStill();
    RobotContainer.LeftFrontSwerveModule.setStill();
    RobotContainer.RightBackSwerveModule.setStill();
    RobotContainer.RightFrontSwerveModule.setStill();

    for(int i=1;i<=4;i++){
      angleGoal[i]=0;
      velocityGoal[i]=0;
    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("angle", swerve_subsystem.get_field_angle());
    double x_value = RobotContainer.stick.getRawAxis(0);
    double y_value = -RobotContainer.stick.getRawAxis(1);
    double rot_value = RobotContainer.stick.getRawAxis(4);

    if (RobotContainer.stick.getRawButtonPressed(1)) {
      if (field_oriented) {
         // Current state is true so turn off
          field_oriented=false;
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 1);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 1);
          Timer.delay(0.1);
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 0);
      } else {
         // Current state is false so turn on
          field_oriented=true;
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 1);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 1);
          Timer.delay(0.3);
          RobotContainer.stick.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.stick.setRumble(RumbleType.kRightRumble, 0);
      }
   }
   //field_oriented=false;
   if(!flag){
     targetangle=swerve_subsystem.get_field_angle();
   }
   //SmartDashboard.putBoolean("field_oriented", field_oriented);
   SmartDashboard.putNumber("targetangle", targetangle);
   if(Math.abs(x_value)<0.1) x_value=0;
   if(Math.abs(y_value)<0.1) y_value=0;
   if(Math.abs(rot_value)<0.1)  rot_value=0;
    if(Math.abs(x_value)<0.1&&Math.abs(y_value)<0.1&&Math.abs(rot_value)<0.1){
      stop_all();
      flag=false;
    }
    else{
      if(Math.abs(rot_value)<0.1) flag=true;
      else flag=false;
      if(field_oriented){
        swerve_subsystem.field_oriented(x_value, y_value, rot_value,Math.toRadians(swerve_subsystem.get_field_angle()));
      }
      else{
        if(flag){
          double error=targetangle-swerve_subsystem.get_field_angle();
          error=-error;
          if(error>180) error-=360;
          else if(error<-180) error+=360;
          rot_value=error*0.005;
          SmartDashboard.putNumber("error", error);
        }
        swerve_subsystem.car_oriented(x_value, y_value, rot_value);
        /*SmartDashboard.putNumber("x_axis", x_value);
        SmartDashboard.putNumber("y_axis", y_value);
        SmartDashboard.putNumber("z_axis", rot_value);*/
      }
      
      angleGoal=swerve_subsystem.get_theta();
      velocityGoal=swerve_subsystem.get_velocity();

      for(int i=1;i<=4;i++){
        angleGoal[i]=(Math.toDegrees(angleGoal[i]))%360;
        velocityGoal[i]=18000*velocityGoal[i]+2000;
      }

      RobotContainer.LeftFrontSwerveModule.setStatus(angleGoal[1], velocityGoal[1]);
      RobotContainer.RightFrontSwerveModule.setStatus(angleGoal[2], velocityGoal[2]);
      RobotContainer.RightBackSwerveModule.setStatus(angleGoal[3], velocityGoal[3]);
      RobotContainer.LeftBackSwerveModule.setStatus(angleGoal[4], velocityGoal[4]);
      /*SmartDashboard.putNumber("angle1", angleGoal[1]);
      SmartDashboard.putNumber("v1", velocityGoal[1]);
      SmartDashboard.putNumber("angle2", angleGoal[2]);
      SmartDashboard.putNumber("v2", velocityGoal[2]);
      SmartDashboard.putNumber("angle3", angleGoal[3]);
      SmartDashboard.putNumber("v3", velocityGoal[3]);
      SmartDashboard.putNumber("angle4", angleGoal[4]);
      SmartDashboard.putNumber("v4", velocityGoal[4]);*/

      //SmartDashboard.putNumberArray("angle", angleGoal);
      //SmartDashboard.putNumberArray("velocity", velocityGoal);
    }
    /*double a=5,v=2000;
    LeftFrontModule.setStatus(a, v);
    RightFrontModule.setStatus(a, v);
    RightBackModule.setStatus(a, v);
    LeftBackModule.setStatus(a, v);*/
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
