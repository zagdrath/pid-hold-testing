/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import static frc.team3602.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.subsystems.CannonSubsystem;

public class RobotContainer {
  // Subsystems
  private final CannonSubsystem cannonSubsys = new CannonSubsystem();

  // Controllers
  private final CommandXboxController xboxController = new CommandXboxController(XBOX_CONTROLLER_PORT);

  // Autonomous
  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    cannonSubsys.setDefaultCommand(cannonSubsys.holdAngle());
  }

  private void configButtonBindings() {
    xboxController.a().onTrue(cannonSubsys.runOnce(() -> cannonSubsys.setAngle(45)));
    xboxController.b().onTrue(cannonSubsys.runOnce(() -> cannonSubsys.setAngle(90)));
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}