/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.team3602.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  // Motor controllers
  private final CANSparkMax rotateMotor = new CANSparkMax(ROTATE_MOTOR_CAN_ID, MotorType.kBrushless);

  // Motor encoders
  private final RelativeEncoder rotateMotorEncoder = rotateMotor.getEncoder();

  // PID controllers
  private final SparkMaxPIDController rotateMotorPIDController = rotateMotor.getPIDController();

  public double angleDegrees;

  public CannonSubsystem() {
    configCannonSubsys();

    resetRotateMotorEncoder();
  }

  @Override
  public void periodic() {
  }

  public double getRotateMotorEncoder() {
    return rotateMotorEncoder.getPosition();
  }

  public void resetRotateMotorEncoder() {
    rotateMotorEncoder.setPosition(0.0);
  }

  public void setAngle(double angleDegrees) {
    this.angleDegrees = angleDegrees;
  }

  public Command holdAngle() {
    return run(() -> {
      rotateMotorPIDController.setReference(angleDegrees, CANSparkMax.ControlType.kPosition);
    });
  }

  private void configCannonSubsys() {
    rotateMotorPIDController.setP(ROTATE_MOTOR_KP);
    rotateMotorPIDController.setI(ROTATE_MOTOR_KI);
    rotateMotorPIDController.setD(ROTATE_MOTOR_KD);
    rotateMotorEncoder.setPositionConversionFactor(360.0 / ROTATE_MOTOR_GEAR_RATIO);
  }
}