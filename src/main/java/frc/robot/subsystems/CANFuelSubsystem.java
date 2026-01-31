// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkMax intake;
  private final RelativeEncoder m_intakeEncoder;
  private final SparkClosedLoopController m_intakePID;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);
    m_intakeEncoder = intake.getEncoder();
    m_intakePID = intake.getClosedLoopController();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(false);
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);

/* Debug start */
    intakeConfig
        .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
        .inverted(ConstantsOffboard.DRIVE_MOTOR_INVERSION)
        .idleMode(IdleMode.kBrake);
    intakeConfig.encoder
        .positionConversionFactor(ConstantsOffboard.DRIVE_ROTATIONS_TO_METERS)
        .velocityConversionFactor(ConstantsOffboard.DRIVE_RPM_TO_METERS_PER_SECOND);
    intakeConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 3
        .p(ConstantsOffboard.DRIVE_KP_PROFILED) //, ClosedLoopSlot.kSlot3)
        .i(ConstantsOffboard.DRIVE_KI_PROFILED) //, ClosedLoopSlot.kSlot3)
        .d(ConstantsOffboard.DRIVE_KD_PROFILED); // ClosedLoopSlot.kSlot3);
        // .outputRange(-1, 1, ClosedLoopSlot.kSlot3);
        // .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          // .kV(12.0 / 5767, ClosedLoopSlot.kSlot3);
    // intakeConfig.closedLoop.maxMotion
    //     .maxAcceleration(20000, ClosedLoopSlot.kSlot3)  // Note: in m/s^2
    //     .cruiseVelocity(10000, ClosedLoopSlot.kSlot3)  // Note: in m/s
    //     .allowedProfileError(1, ClosedLoopSlot.kSlot3);

    // m_driveMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_driveEncoder.setPosition(0);

/* Debug end */
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Floor intake roller value", FLOOR_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setIntake(double voltage) {
    // intake.setVoltage(voltage);
    m_intakePID.setSetpoint(voltage, ControlType.kVelocity); //, ClosedLoopSlot.kSlot3);
    SmartDashboard.putNumber("Intake Setpoint", voltage);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Velocity", m_intakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Position", m_intakeEncoder.getPosition());
  }
}
