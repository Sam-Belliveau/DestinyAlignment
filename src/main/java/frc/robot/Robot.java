package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.Logitech;
import com.stuypulse.stuylib.input.gamepads.NetKeyGamepad;
import com.stuypulse.stuylib.math.*;
import com.stuypulse.stuylib.streams.*;
import com.stuypulse.stuylib.streams.filters.*;
import com.stuypulse.stuylib.network.limelight.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX leftRearMotor;
  private WPI_TalonSRX rightRearMotor;
  private SpeedControllerGroup leftSpeedController;
  private SpeedControllerGroup rightSpeedController;
  private DifferentialDrive differentialDrive;

  private WPI_VictorSPX shooterMotor;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Limelight.setLEDMode(Limelight.LEDMode.FORCE_OFF);
    
    leftFrontMotor = new WPI_TalonSRX(1);
    rightFrontMotor = new WPI_TalonSRX(3);
    leftRearMotor = new WPI_TalonSRX(4);
    rightRearMotor = new WPI_TalonSRX(2);
    leftFrontMotor.setInverted(true);
    rightFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    rightRearMotor.setInverted(true);
    leftSpeedController = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
    rightSpeedController = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);
    differentialDrive = new DifferentialDrive(leftSpeedController, rightSpeedController);

    shooterMotor = new WPI_VictorSPX(13);

    resetSmartDashboard();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
        break;
    }
  }

  private double toFeet(int feet, double inches) {
    return ((double)feet) + (inches / 12.0);
  }

  public Gamepad gamepad = new Logitech.XMode(0);

  public IStream mRawSpeed = () -> gamepad.getRawRightTriggerAxis() - gamepad.getRawLeftTriggerAxis();
  public IStream mRawAngle = () -> gamepad.getLeftX();

  public IStream mSpeed = new FilteredIStream(mRawSpeed,
    new RollingAverage(24)
  );

  public IStream mAngle = new FilteredIStream(mRawAngle,
    new RollingAverage(12)
  );

  // If the robot is aligning
  public boolean mAutoMode = false;

  // Used to get derivative speed and angle
  public double mLastAutoDist = 0;
  public double mLastAutoAngle = 0;

  // Feet, units dont matter as long as they are consistant
  public double kGoalHeight = toFeet(7, 7);

  // How far away from the goal we want to be
  public double kTargetDistance = toFeet(10, 0);

  // Information about the Limelights Position
  public double kLimelightHeight = toFeet(2, 7); // ft
  public double kLimelightDistance = toFeet(2, 0); // ft from shooter
  public double kLimelightPitch = 28; // deg
  public double kLimelightAngle = 0; // deg
  
  // Information for aligning algorithms
  public double kDistanceError = 0.2;
  public double kDistance_P = 0.6;
  public double kDistance_D = 0.3;
  public double kAngleError = 2;
  public double kAngle_P = 0.1;
  public double kAngle_D = 0.4;

  // Amount that user has control over the robot when aligning
  public double kUserBias = 0.3;

  public void resetSmartDashboard() {
    SmartDashboard.putNumber("Goal Height", kGoalHeight);
    
    SmartDashboard.putNumber("Target Goal Distance", kTargetDistance);
    
    SmartDashboard.putNumber("Limelight Height", kLimelightHeight);
    SmartDashboard.putNumber("Limelight Distance", kLimelightDistance);
    SmartDashboard.putNumber("Limelight Pitch", kLimelightPitch);
    SmartDashboard.putNumber("Limelight Angle", kLimelightAngle);
    
    SmartDashboard.putNumber("Max Distance Error", kDistanceError);
    SmartDashboard.putNumber("Distance_P", kDistance_P);
    SmartDashboard.putNumber("Distance_D", kDistance_D);

    SmartDashboard.putNumber("Max Angle Error", kAngleError);
    SmartDashboard.putNumber("Angle_P", kAngle_P);
    SmartDashboard.putNumber("Angle_D", kAngle_D);

    SmartDashboard.putNumber("User Bias", kUserBias);
  }

  public void getSmartDashboard() {
    kGoalHeight = SmartDashboard.getNumber("Goal Height", kGoalHeight);
    
    kTargetDistance = SmartDashboard.getNumber("Target Goal Distance", kTargetDistance);

    kLimelightHeight = SmartDashboard.getNumber("Limelight Height", kLimelightHeight);
    kLimelightDistance = SmartDashboard.getNumber("Limelight Distance", kLimelightDistance);
    kLimelightPitch = SmartDashboard.getNumber("Limelight Pitch", kLimelightPitch);
    kLimelightAngle = SmartDashboard.getNumber("Limelight Angle", kLimelightAngle);

    kDistanceError = SmartDashboard.getNumber("Max Distance Error", kDistanceError);
    kDistance_P = SmartDashboard.getNumber("Distance_P", kDistance_P);
    kDistance_D = SmartDashboard.getNumber("Distance_D", kDistance_D);

    kAngleError = SmartDashboard.getNumber("Max Angle Error", kAngleError);
    kAngle_P = SmartDashboard.getNumber("Angle_P", kAngle_P);
    kAngle_D = SmartDashboard.getNumber("Angle_D", kAngle_D);

    kUserBias = SmartDashboard.getNumber("User Bias", kUserBias);
  }

  @Override
  public void teleopPeriodic() {
    double speed = mSpeed.get();
    double angle = mAngle.get();

    // Reset Settings
    if(gamepad.getRawRightButton()) {
      resetSmartDashboard();
    }

    // Toggle auto mode with button
    if(gamepad.getRawTopButton()) {
      mAutoMode = !mAutoMode;

      // Wait for button to be released
      while(gamepad.getRawTopButton()) {}
    }
 
    // Check if allignment button is pressed
    if(gamepad.getRawLeftButton() || mAutoMode) {
      getSmartDashboard();
      Limelight.setLEDMode(Limelight.LEDMode.FORCE_ON);

      double auto_speed = 0;
      double auto_angle = 0;
      
      if(Limelight.hasValidTarget()) {
        // Get the angle and pitch of the target
        double goal_angle = Limelight.getTargetXAngle() + kLimelightAngle;
        double goal_pitch = Limelight.getTargetYAngle() + kLimelightPitch;

        // Get the height of the goal reletive to the limelight
        double goal_height = kGoalHeight - kLimelightHeight;

        // Get the distance of the the target from the limelight using geometry
        double goal_dist = goal_height / Math.tan(Math.toRadians(goal_pitch)) - kLimelightDistance;

        // Get the distance that the robot needs to travel
        double dist_error = goal_dist - kTargetDistance;

        // Get derivatives of speed and angle
        double dist_change  = (mLastAutoDist  - dist_error);
        double angle_change = (mLastAutoAngle - goal_angle);
        mLastAutoDist  = dist_error;
        mLastAutoAngle = goal_angle;

        SmartDashboard.putNumber("Target X Angle", goal_angle);
        SmartDashboard.putNumber("Target Y Angle", goal_pitch);

        SmartDashboard.putNumber("Target Distance", goal_dist);
        SmartDashboard.putNumber("Target Error", dist_error);

        // If the angle is good, then start moving
        if((Math.abs(goal_angle) < kAngleError) && (Math.abs(dist_error) > kDistanceError)) {
          auto_speed += dist_error * kDistance_P;
          auto_angle -= dist_change * kDistance_D;
        } 

        // Otherwise start turning towards the target
        else {
          auto_angle += goal_angle * kAngle_P;
          auto_angle -= angle_change * kAngle_D;
        }
      }
      
      // Limit speed and angle to prevent the alignment algorithm
      // from overriding the user inputs / give crazy values
      auto_speed = SLMath.limit(auto_speed, -1, 1);
      auto_angle = SLMath.limit(auto_angle, -1, 1);
      speed = SLMath.limit(speed, -1, 1);
      angle = SLMath.limit(angle, -1, 1);

      // Average user controls with alignment
      speed = (speed * kUserBias) + (auto_speed * (1 - kUserBias));
      angle = (angle * kUserBias) + (auto_angle * (1 - kUserBias));
    } else {
      Limelight.setLEDMode(Limelight.LEDMode.FORCE_OFF);
    }

    differentialDrive.arcadeDrive(speed, angle, false);
  }

  @Override
  public void testPeriodic() {
  }
}
