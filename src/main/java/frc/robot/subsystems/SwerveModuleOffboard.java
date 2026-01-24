// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModuleOffboard {
  // Drive motor
  private final SparkMax m_driveMotor;
  private final RelativeEncoder m_driveEncoder;
  private final SparkClosedLoopController m_drivePID;

  // private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  // private final PositionVoltage turnPosition =  new PositionVoltage(0);

  private static final SparkMaxConfig driveConfig = new SparkMaxConfig();
  // private static final CANcoderConfiguration testCanCoderConfig = new CANcoderConfiguration();
  // private static final Slot0Configs rotationConfigPID = rotationConfig.Slot0;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.32, 1.51, 0.27);

  // Turning motor
  private final SparkMax m_turningMotor;
  private final RelativeEncoder m_turningEncoder;
  private final SparkClosedLoopController m_turningPID;
  
  private static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  // Swerve module absolute encoder (wheel angle)
  private final CANcoder m_canCoder;
  private final double m_canCoderOffsetDegrees;
  private double lastAngle;

  SwerveModuleState state;
  private int DisplayCount = 0;

  /**
   * SwerveModuleOffboard - A SparkMax-based swerve module with canCoder wheel angle measurement
   *
   * @param driveMotorID The CAN ID of the drive motor.
   * @param turningMotorID The CAN ID of the turning motor.
   * @param magEncoderID The CAN ID of the magnetic encoder.
   * @param magEncoderOffsetDegrees The absolute offset of the magnetic encoder.
   */
  public SwerveModuleOffboard(int driveMotorID, int turningMotorID, int magEncoderID,
      double magEncoderOffsetDegrees) {
    // Create drive motor objects
    m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePID = m_driveMotor.getClosedLoopController();

    // Create turning motor objects
    m_turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningPID = m_turningMotor.getClosedLoopController();

    // Create steering encoder objects (high resolution encoder)
    m_canCoder = new CANcoder(magEncoderID, "rio");
    m_canCoderOffsetDegrees = magEncoderOffsetDegrees;
    SignalLogger.stop();

    driveConfig
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);
    driveConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(ConstantsOffboard.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 3
        .p(0.0001, ClosedLoopSlot.kSlot3)
        .i(0, ClosedLoopSlot.kSlot3)
        .d(0, ClosedLoopSlot.kSlot3)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot3)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot3);
    driveConfig.closedLoop.maxMotion
        .maxAcceleration(500, ClosedLoopSlot.kSlot3)
        .cruiseVelocity(6000, ClosedLoopSlot.kSlot3)
        .allowedProfileError(1, ClosedLoopSlot.kSlot3);

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    turningConfig
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);
    turningConfig.encoder
        .positionConversionFactor(ConstantsOffboard.ANGLE_RPM_TO_RADIANS_PER_SECOND)
        .velocityConversionFactor(1);
    turningConfig.closedLoop
        // Set PID values for position control in slot 2
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1, ClosedLoopSlot.kSlot2)
        .i(0, ClosedLoopSlot.kSlot2)
        .d(0, ClosedLoopSlot.kSlot2)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot2);
        // .feedForward
        //   // kV is now in Volts, so we multiply by the nominal voltage (12V)
        //   .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
    turningConfig.closedLoop.maxMotion
        .cruiseVelocity(1000, ClosedLoopSlot.kSlot2)
        .maxAcceleration(1000, ClosedLoopSlot.kSlot2)
        .allowedProfileError(1, ClosedLoopSlot.kSlot2);

    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    configureDevices();

    lastAngle = getState().angle.getRadians();

    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    state = desiredState;
    if (SwerveConstants.DISABLE_ANGLE_OPTIMIZER) {
      // Only enable when correcting wheel offsets!
      if (DisplayCount++ > 40) {
        System.out.println("CAUTION: Steering optimizer is disabled!");
        DisplayCount = 0;
      }
    } else {
      // Optimize the reference state to avoid spinning further than 90 degrees
      // state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));
      // m_turningEncoder.setPosition(Units.degreesToRadians(m_canCoder.getAbsolutePosition().getValueAsDouble() * 360.0 - m_canCoderOffsetDegrees));
      // m_turningEncoder.setPosition(Units.rotationsToRadians(m_turningEncoder.getPosition().getValueAsDouble()));
      // state.optimize(new Rotation2d(m_turningEncoder.getPosition()));
      // state.angle = Rotation2d.fromDegrees(wrapTo360(state.angle.getDegrees()));
      // state.angle = Rotation2d.fromDegrees(MathUtil.inputModulus(state.angle.getDegrees(), 0, 360.0));
      // optimize(state.angle.getDegrees(), Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble()).getDegrees());
      state.optimize(Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble()));
    }

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromRotations(m_turningEncoder.getPosition())).getCos();

    // Set the PID reference states
    // driveVelocity.Velocity = (state.speedMetersPerSecond * 60) / Constants.ConstantsOffboard.WHEEL_CIRCUMFERENCE;
    // driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
    // m_driveMotor.setControl(driveVelocity);
    m_drivePID.setSetpoint(state.speedMetersPerSecond, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot3);
    // turnPosition.Position = (state.angle.getRotations());
    // m_turningMotor.setControl(turnPosition.withEnableFOC(false).withPosition(turnPosition.Position));
    // m_turningPID.setReference(state.angle.getRadians(), (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) ? SparkMax.ControlType.kMAXMotionPositionControl : SparkMax.ControlType.kPosition);
    m_turningPID.setSetpoint(state.angle.getRadians(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot2);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoder() {
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    double velocity = m_driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(m_turningEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public static double wrapTo360(double angleDeg) {
    angleDeg %= 360.0;
    if (angleDeg < 0) {
        angleDeg += 360.0;
    }
    return angleDeg;
  }

  public void optimize(double goalAngle, double currentAngle) {
    double x = goalAngle - currentAngle;
    double y = (goalAngle + 360) - currentAngle;
    double z = goalAngle - (currentAngle + 360);
    if (Math.abs(x) > 90.0 && Math.abs(y) > 90.0 && Math.abs(z) > 90.0) {
      state.speedMetersPerSecond *= -1;
      state.angle = Rotation2d.fromDegrees(goalAngle).rotateBy(Rotation2d.kPi);
      // Constants.yCheck = true;
    } 
    else if (Math.abs(x) > 90.0 && Math.abs(y) <= 90.0) {
      state.angle = Rotation2d.fromDegrees((goalAngle + 360));
      Constants.yCheck = true;
    }
    else if (Math.abs(x) > 90.0 && Math.abs(z) <= 90.0) {
      state.angle = Rotation2d.fromDegrees((goalAngle - 360));
      Constants.yCheck = true;
    }
  }

  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getCanCoder() {
    var posVal = m_canCoder.getPosition();
    if(posVal.getStatus().isOK()) {
        double val = posVal.getValueAsDouble();
        return val * 360.0;
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0.0;
    }
  }

  /**
   * Returns the SparkMax internal encoder's measured turn angle in degrees.
   */
  public double getAngle() {
    return Units.radiansToDegrees(m_turningEncoder.getPosition());
  }

  /**
   * Returns the SparkMax internal encoder's measured position in meters.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(m_turningEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  // public double getDesiredPosition() {
  //   return turnPosition.Position;
  // }
  
  public double getCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

   public double getTurnCurrent() {
    return m_turningMotor.getOutputCurrent();
  }

  public double getWheelAngleDegrees() {
    return (m_canCoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  private void configureDevices() {
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    toApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    toApply.MagnetSensor.MagnetOffset = m_canCoderOffsetDegrees;
    m_canCoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    m_canCoder.getPosition().setUpdateFrequency(100);
    m_canCoder.getVelocity().setUpdateFrequency(100);

    // According to this:
    // https://www.chiefdelphi.com/t/ctre-phoenix-pro-to-phoenix-6-looking-back-and-looking-ahead/437313/27
    // When using Phoenix6, the CanCoder should not have problems on startup as long as we wait for an update and check for errors.
    var posVal = m_canCoder.getAbsolutePosition().waitForUpdate(0.1); // This actaully waits that long! Don't call after init!
    if(posVal.getStatus().isOK()) {
        /* Perform seeding */
        double val = posVal.getValueAsDouble();
        m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0 - m_canCoderOffsetDegrees));
        // m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0));
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
    }

    // Math check! Is reading the SparkMax position good enough?
    // CANcoder resolution in degrees: 360/4096 = 0.088 degrees
    // SparkMax encoder resolution: 360 / ( 48 positions * ANGLE_GEAR_RATIO (150/7) ) = 0.35 degrees
    // So is the CANcoder slightly better? Yes, but the CANcoder hardware spec:
    // https://store.ctr-electronics.com/content/user-manual/CANCoder%20User's%20Guide.pdf
    // says that the absolute position can be off by 1.44 degrees if there is rotation!
    // It may be possible to servo to m_canCoder.getPosition(), but then then CAN bus utilization will go way up.
    // Future project: Servo on the CANcoder position and see if CAN bus utilization is a problem.
  }
}