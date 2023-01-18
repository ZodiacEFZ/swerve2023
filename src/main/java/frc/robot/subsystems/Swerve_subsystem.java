package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve_subsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  AHRS ahrs;
  public Swerve_subsystem() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

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
  public void periodic() {
    // This method will be called once per scheduler run
  }

  double s2=Math.sqrt(2),maxv=1;
  double[] velocity_goal=new double[8],theta_goal=new double[8];
  public void field_oriented(double m,double f,double r,double a){
    double sin=Math.sin(a),cos=Math.cos(a);
    double m1=m*cos+f*sin,f1=f*cos-m*sin;
    car_oriented(m1,f1,r);
  }
  public double zero(double x){
    if(Math.abs(x)<0.01) return 0;
    return x;
  }

  public void car_oriented(double m,double f,double r){
    double k,Vm=0;
    double v1=Math.sqrt((m-r/s2)*(m-r/s2)+(f-r/s2)*(f-r/s2));
    double v2=Math.sqrt((m-r/s2)*(m-r/s2)+(f+r/s2)*(f+r/s2));
    double v3=Math.sqrt((m+r/s2)*(m+r/s2)+(f+r/s2)*(f+r/s2));
    double v4=Math.sqrt((m+r/s2)*(m+r/s2)+(f-r/s2)*(f-r/s2));
    Vm=Math.max(Math.max(v1, v2), Math.max(v3, v4));
    if(Vm>maxv)
    {
      k=maxv/Vm;
      m*=k;f*=k;r*=k;
    } 
    
    v1=Math.sqrt((m-r/s2)*(m-r/s2)+(f-r/s2)*(f-r/s2));
    v2=Math.sqrt((m-r/s2)*(m-r/s2)+(f+r/s2)*(f+r/s2));
    v3=Math.sqrt((m+r/s2)*(m+r/s2)+(f+r/s2)*(f+r/s2));
    v4=Math.sqrt((m+r/s2)*(m+r/s2)+(f-r/s2)*(f-r/s2));

    double theta1=Math.atan2(zero(f-r/s2),zero(m-r/s2));
    double theta2=Math.atan2(zero(f+r/s2),zero(m-r/s2));
    double theta3=Math.atan2(zero(f+r/s2),zero(m+r/s2));
    double theta4=Math.atan2(zero(f-r/s2),zero(m+r/s2));

    if(theta1<0)  theta1+=2*Math.PI;
    if(theta2<0)  theta2+=2*Math.PI;
    if(theta3<0)  theta3+=2*Math.PI;
    if(theta4<0)  theta4+=2*Math.PI;
    
    velocity_goal[1]=v1;velocity_goal[2]=v2;velocity_goal[3]=v3;velocity_goal[4]=v4;
    theta_goal[1]=theta1;theta_goal[2]=theta2;theta_goal[3]=theta3;theta_goal[4]=theta4;
    //do something with the data
    
    // SmartDashboard.putNumberArray("v_goal", velocity_goal);
    // SmartDashboard.putNumberArray("angle_goal", theta_goal);
  }

  public double[] get_theta(){
    return theta_goal;
  }
  public double[] get_velocity(){
    return velocity_goal;
  }
  public double get_field_angle(){
    double angle=ahrs.getYaw();
    angle=-angle;
    angle=(angle%360+360)%360;
    //angle=Math.toRadians(angle);
    return angle;
    //return Math.PI/6;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}