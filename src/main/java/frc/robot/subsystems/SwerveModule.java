package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  int number;
  double zeroPosition;
  double kVelocity = 1;

  double theta_position;
  double position;

  public WPI_TalonSRX angleMotor;
  public WPI_TalonFX velocityMotor;

  public SwerveModule(int number, int angleMotorPort, int velocityMotorPort, double zeroPosition, int inverse) {
    this.number = number;
    this.zeroPosition = zeroPosition;
    
    angleMotor = new WPI_TalonSRX(angleMotorPort);

    angleMotor.setInverted(false);
    angleMotor.configFactoryDefault(); 
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0); 
    angleMotor.config_kP(0, 0.8);       // *PID need improvement
    angleMotor.config_kI(0, 0); 
    angleMotor.config_kD(0, 0); 
    angleMotor.config_kF(0, 0); 
    angleMotor.setNeutralMode(NeutralMode.Coast); 
    angleMotor.setSensorPhase(true);

    velocityMotor = new WPI_TalonFX(velocityMotorPort);

    if(inverse == 0)
      velocityMotor.setInverted(false);             //same direction as zeroPosition
    else
      velocityMotor.setInverted(true);
    velocityMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    velocityMotor.configFactoryDefault(); 
    // velocityMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0); 
    velocityMotor.config_kP(0, 0.1);        // *PID need improvement
    velocityMotor.config_kI(0, 0); 
    velocityMotor.config_kD(0, 0); 
    //velocityMotor.config_kF(0, 0); 
    velocityMotor.setNeutralMode(NeutralMode.Coast); 
    velocityMotor.setSensorPhase(true); 
    //SmartDashboard.putNumber("v",velocityMotor.getSelectedSensorVelocity());
  }

  public void setStatus(double angleGoal, double velocityGoal){
    double raw_error = angleGoal - theta_position;
    double theta_change = 0;

    if(Math.abs(raw_error)<90) {
      theta_change = raw_error;
      kVelocity = 1;
    }
    if(raw_error>=90&&raw_error<270){
      theta_change=raw_error-180;
      kVelocity=-1;
    }
    if(raw_error>=270&&raw_error<=360){
      theta_change=raw_error-360;
      kVelocity=1;
    }
    if(raw_error<-90&&raw_error>=-270){
      theta_change=raw_error+180;
      kVelocity=-1;
    }
    if(raw_error<-270&&raw_error>=-360){
      theta_change=raw_error+360;
      kVelocity=1;
    }
    //SmartDashboard.putNumber("theta_change " + number, theta_change);
    //SmartDashboard.putNumber("position_goal " + number, positionGoal);

    double positionGoal = theta_change/360*4096 + position;

    if(Math.abs(theta_change)>0.5)       // * Dead ban of theta change need improvement
      angleMotor.set(ControlMode.Position, positionGoal);


    // velocityMotor.set(ControlMode.Velocity, kVelocity*velocityGoal/2);
    velocityMotor.set(ControlMode.Velocity, kVelocity*velocityGoal);
    //SmartDashboard.putNumber("velocityGoal " + number, kVelocity*velocityGoal);

  }

  public void setStill(){
    angleMotor.set(0);
    velocityMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("zero_position " + number, zeroPosition);
    // velocityMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    position = angleMotor.getSelectedSensorPosition();
    theta_position = (((position - zeroPosition)/4096*360)%360+360)%360;
    //SmartDashboard.putNumber("Position" + number, angleMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("theta_position " + number, theta_position);
    SmartDashboard.putNumber("SwerveModule Velocity" + number, velocityMotor.getSelectedSensorVelocity());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}