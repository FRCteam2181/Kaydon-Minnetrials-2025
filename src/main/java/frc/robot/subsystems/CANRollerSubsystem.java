// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotorTop;
  private final SparkMax rollerMotorBottom;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotorTop = new SparkMax(RollerConstants.TOP_ROLLER_MOTOR_ID, MotorType.kBrushed);
    rollerMotorBottom = new SparkMax(RollerConstants.BOTTOM_ROLLER_MOTOR_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotorTop.setCANTimeout(250);
    rollerMotorBottom.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    rollerMotorTop.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotorBottom.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRollerSpitter(double forward, double reverse) {
    rollerMotorTop.set(forward - reverse);
    rollerMotorBottom.set(forward - reverse);
  }

  public void runRollerSucker(double forward, double reverse) {
    rollerMotorBottom.set(forward - reverse);
  }
}