/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.team3602.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  // Motor controllers
  private final CANSparkMax rotateMotor = new CANSparkMax(ROTATE_MOTOR_CAN_ID, MotorType.kBrushless);

  // Motor encoders
  private final RelativeEncoder rotateMotorEncoder = rotateMotor.getEncoder();

  // PID controllers
  private final PIDController rotateMotorPIDController = new PIDController(ROTATE_MOTOR_KP, ROTATE_MOTOR_KI,
      ROTATE_MOTOR_KD);

  public CannonSubsystem() {
    configCannonSubsys();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotate Motor Setpoint", rotateMotorPIDController.getSetpoint());
    SmartDashboard.putNumber("Rotate Motor Encoder", getRotateMotorEncoder());
  }

  public double getRotateMotorEncoder() {
    return rotateMotorEncoder.getPosition();
  }

  public void setAngle(double angleDegrees) {
    rotateMotorPIDController.reset();
    rotateMotorPIDController.setSetpoint(angleDegrees);
  }

  public Command holdAngle() {
    return run(() -> {
      rotateMotor.set(rotateMotorPIDController.calculate(getRotateMotorEncoder()));
    });
  }

  private void configCannonSubsys() {
    rotateMotorEncoder.setPositionConversionFactor(360 / ROTATE_MOTOR_GEAR_RATIO);
  }
}
