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
import com.stuypulse.stuylib.control.PIDController;

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
    return ((double) feet) + (inches / 12.0);
  }

  public Gamepad gamepad = new Logitech.XMode(0);

  public IStream mRawSpeed = () -> gamepad.getRawRightTriggerAxis() - gamepad.getRawLeftTriggerAxis();
  public IStream mRawAngle = () -> gamepad.getLeftX();

  public IStream mSpeed = new FilteredIStream(mRawSpeed, new RollingAverage(48));
  public IStream mAngle = new FilteredIStream(mRawAngle, new RollingAverage(24));

  // Feet, units dont matter as long as they are consistant
  public double kGoalHeight;

  // How far away from the goal we want to be
  public double kTargetDistance;

  // Information about the Limelights Position
  public double kLimelightHeight; // ft
  public double kLimelightDistance; // ft from shooter
  public double kLimelightPitch; // deg
  public double kLimelightAngle; // deg

  // Information for aligning algorithms
  public double kDistance_P;
  public double kDistance_I;
  public double kDistance_D;

  public double kAngleError;
  public double kAngle_P;
  public double kAngle_I;
  public double kAngle_D;

  // Amount that user has control over the robot when aligning
  public double kUserBias;

  public void resetSmartDashboard() {

    // Feet, units dont matter as long as they are consistant
    SmartDashboard.putNumber("Goal Height", kGoalHeight = toFeet(7, 6));

    // How far away from the goal we want to be
    SmartDashboard.putNumber("Target Goal Distance", kTargetDistance = toFeet(10, 0));

    // Information about the Limelights Position
    SmartDashboard.putNumber("Limelight Height", kLimelightHeight = toFeet(2, 7));
    SmartDashboard.putNumber("Limelight Distance", kLimelightDistance = toFeet(3, 0));
    SmartDashboard.putNumber("Limelight Pitch", kLimelightPitch = 17);
    SmartDashboard.putNumber("Limelight Angle", kLimelightAngle = 0);

    // Information for aligning algorithms

    // Distance Smoothing (rolling average 24)
    // Speed Smoothing (rolling average 12)
    SmartDashboard.putNumber("Distance_P", kDistance_P = 0.36);
    SmartDashboard.putNumber("Distance_I", kDistance_I = 0.01);
    SmartDashboard.putNumber("Distance_D", kDistance_D = 0.06);

    // Angle Error Smoothing (n / a)
    // Angle Smoothing (rolling average 6)
    SmartDashboard.putNumber("Max Angle Error", kAngleError = 2);
    SmartDashboard.putNumber("Angle_P", kAngle_P = 0.16);
    SmartDashboard.putNumber("Angle_I", kAngle_I = 0.01);
    SmartDashboard.putNumber("Angle_D", kAngle_D = 0.015);

    SmartDashboard.putNumber("User Bias", kUserBias = 0.0);
  }

  public void getSmartDashboard() {
    kGoalHeight = SmartDashboard.getNumber("Goal Height", kGoalHeight);

    kTargetDistance = SmartDashboard.getNumber("Target Goal Distance", kTargetDistance);

    kLimelightHeight = SmartDashboard.getNumber("Limelight Height", kLimelightHeight);
    kLimelightDistance = SmartDashboard.getNumber("Limelight Distance", kLimelightDistance);
    kLimelightPitch = SmartDashboard.getNumber("Limelight Pitch", kLimelightPitch);
    kLimelightAngle = SmartDashboard.getNumber("Limelight Angle", kLimelightAngle);

    kDistance_P = SmartDashboard.getNumber("Distance_P", kDistance_P);
    kDistance_I = SmartDashboard.getNumber("Distance_I", kDistance_I);
    kDistance_D = SmartDashboard.getNumber("Distance_D", kDistance_D);

    kAngleError = SmartDashboard.getNumber("Max Angle Error", kAngleError);
    kAngle_P = SmartDashboard.getNumber("Angle_P", kAngle_P);
    kAngle_I = SmartDashboard.getNumber("Angle_I", kAngle_I);
    kAngle_D = SmartDashboard.getNumber("Angle_D", kAngle_D);

    kUserBias = SmartDashboard.getNumber("User Bias", kUserBias);

    mAnglePID.setP(kAngle_P);
    mAnglePID.setI(kAngle_I);
    mAnglePID.setD(kAngle_D);

    mDistancePID.setP(kDistance_P);
    mDistancePID.setI(kDistance_I);
    mDistancePID.setD(kDistance_D);
  }

  public PIDController mAnglePID = new PIDController(kAngle_P, kAngle_I, kAngle_D);
  public PIDController mDistancePID = new PIDController(kDistance_P, kDistance_I, kDistance_D);

  public IStreamFilter mDistanceFilter = new RollingAverage(24);
  public IStreamFilter mSpeedPIDFilter = new RollingAverage(12);
  public IStreamFilter mAnglePIDFilter = new RollingAverage(6);

  @Override
  public void teleopPeriodic() {
    double speed = mSpeed.get();
    double angle = mAngle.get();

    // Reset Settings
    if (gamepad.getRawRightButton()) {
      resetSmartDashboard();
    }

    // Check if allignment button is pressed
    if (gamepad.getRawLeftButton()) {
      getSmartDashboard();
      Limelight.setLEDMode(Limelight.LEDMode.FORCE_ON);

      double auto_speed = 0;
      double auto_angle = 0;

      if (Limelight.hasValidTarget()) {
        // Get the angle and pitch of the target
        double goal_angle = Limelight.getTargetXAngle() + kLimelightAngle;
        double goal_pitch = Limelight.getTargetYAngle() + kLimelightPitch;
    
        // Get the height of the goal reletive to the limelight
        double goal_height = kGoalHeight - kLimelightHeight;
    
        // Get the distance of the the target from the limelight using geometry
        double goal_dist = goal_height / Math.tan(Math.toRadians(goal_pitch)) - kLimelightDistance;
    
        SmartDashboard.putNumber("Target Distance", goal_dist);
        SmartDashboard.putNumber("Target Error", goal_dist - kTargetDistance);
        SmartDashboard.putNumber("Target X Angle", goal_angle);
        SmartDashboard.putNumber("Target Y Angle", goal_pitch);

        // Get PID on distance and angle
        auto_speed = mDistancePID.update(goal_dist, kTargetDistance);
        auto_angle = mAnglePID.update(goal_angle, 0.0);

        // If the angle is good, then start moving
        if (Math.abs(mAnglePID.getError()) > kAngleError) {
          auto_speed *= 0.01;
        }
      }

      // Limit speed and angle to prevent the alignment algorithm
      // from overriding the user inputs / give crazy values
      auto_speed = mSpeedPIDFilter.get(SLMath.limit(auto_speed, -1, 1));
      auto_angle = mAnglePIDFilter.get(SLMath.limit(auto_angle, -1, 1));
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
