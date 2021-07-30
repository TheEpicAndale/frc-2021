// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RunFlywheel extends CommandBase {
    public static double endVoltage;
    /** Creates a new RunFlywheel. */
    private double initTime;
    public static double desiredRPM;
    private double voltage = 0;
    private double switchVoltage;
    private boolean hasSwitched = false;
    private double voltageIncrement = 0.05;
    private double rampDampener = 0;

    private boolean quitRamp = false;

    private Button maxRPMSetting;
    private Button minRPMSetting;
    private Button stopButton;
    private POVButton dPadUpButton;
    private POVButton dPadDownButton;
    private double RPMError = 0;
    private double previousTime = 0;
    private double deltaTime = 0;
    private double errorSum = 0;
    private double integralRadius = 100;
    private double previousError = 0;
    private double errorRate = 0;
    private final double kP = 0.0002;
    private final double kI = 0.0002;
    private final double kD = 0.001;


    public RunFlywheel() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Shooter.flywheel);
        dPadUpButton = new POVButton(Robot.joystick, 0);
        dPadDownButton = new POVButton(Robot.joystick, 180);
        // maxRPMSetting = new JoystickButton(Robot.joystick, Constants.TEST_FLYWHEEL_UP);
        // minRPMSetting = new JoystickButton(Robot.joystick, Constants.TEST_FLYWHEEL_DOWN);
        stopButton = new JoystickButton(Robot.joystick, Constants.FUNNEL_BUTTON);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        if (Shooter.flywheel.shootIteration == 1) {
            desiredRPM = 4000;
        } else if (Shooter.flywheel.shootIteration == 2) {
            desiredRPM = 4250;
        } else if (Shooter.flywheel.shootIteration == 3) {
            desiredRPM = 4250;
        } else {
            desiredRPM = 4000;
        }
        // desiredRPM = 3500; // For shooting tests, change RPM
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If statement to increment RPM for testing without restarting robot - ONLY FOR
        // TESTING - Needs to be able to be run in between shooting balls? -> Repeat
        // ShootCell
        RPMError = desiredRPM - Shooter.flywheel.getRPM();
        deltaTime = Timer.getFPGATimestamp() - previousTime;
        if(Math.abs(RPMError) < integralRadius) {
            errorSum = RPMError * deltaTime;
        }
        errorRate = ((RPMError - previousError) / deltaTime);
        
        voltage = kP * RPMError + kI * errorSum + kD * errorRate;
        
        if (dPadUpButton.get()) {
            desiredRPM = 4850;
        } else if (dPadDownButton.get()) {
            desiredRPM = 4250;
        }

        if (Math.abs(Shooter.flywheel.getRPM() - desiredRPM) <= 35
                || (hasSwitched == false && Shooter.flywheel.getRPM() > desiredRPM)) {
            switchVoltage = voltage;
            initTime = Timer.getFPGATimestamp();
            // System.out.println("Timer reset!");
            hasSwitched = true;
        }

        // if (Math.abs(Shooter.flywheel.getRPM() - desiredRPM) <= 100) {
        //     rampDampener = 0.03 * (Timer.getFPGATimestamp() - initTime);
        // } else {
        //     rampDampener = 0;
        // }

        // if (hasSwitched == false) {
        //     if (Shooter.flywheel.getRPM() < desiredRPM) {
        //         if (Shooter.flywheel.shootIteration >= 1) {
        //             voltage = 0.1 + voltageIncrement * (Timer.getFPGATimestamp() - initTime) - rampDampener;
        //             // } else if (Shooter.flywheel.shootIteration > 1) {
        //             // voltage = 0.1 + 0.02 * (Timer.getFPGATimestamp() - initTime);
        //         } else {
        //             System.out.println("Shoot Iteration is wrong!");
        //             quitRamp = true;
        //         }
        //     } else if (Shooter.flywheel.getRPM() > desiredRPM) {
        //         if (Shooter.flywheel.shootIteration >= 1) {
        //             voltage = 0.1 - voltageIncrement * (Timer.getFPGATimestamp() - initTime) - rampDampener;
        //             // } else if (Shooter.flywheel.shootIteration > 1) {
        //             // voltage = 0.1 + 0.02 * (Timer.getFPGATimestamp() - initTime);
        //         } else {
        //             System.out.println("Shoot Iteration is wrong!");
        //             quitRamp = true;
        //         }
        //     } else {
        //         System.out.println("This message should not print");
        //         voltage = 0;
        //     }
        // } else {
        //     if (Shooter.flywheel.getRPM() < desiredRPM) {
        //         if (Shooter.flywheel.shootIteration >= 1) {
        //             voltage = switchVoltage + voltageIncrement * (Timer.getFPGATimestamp() - initTime) - rampDampener;
        //             // } else if (Shooter.flywheel.shootIteration > 1) {
        //             // voltage = 0.1 + 0.02 * (Timer.getFPGATimestamp() - initTime);
        //         } else {
        //             System.out.println("Shoot Iteration is wrong!");
        //             quitRamp = true;
        //         }
        //     } else if (Shooter.flywheel.getRPM() > desiredRPM) {
        //         if (Shooter.flywheel.shootIteration >= 1) {
        //             voltage = switchVoltage - voltageIncrement * (Timer.getFPGATimestamp() - initTime) - rampDampener;
        //             // } else if (Shooter.flywheel.shootIteration > 1) {
        //             // voltage = 0.1 + 0.02 * (Timer.getFPGATimestamp() - initTime);
        //         } else {
        //             System.out.println("Shoot Iteration is wrong!");
        //             quitRamp = true;
        //         }
        //     } else {
        //         System.out.println("This message should not print");
        //         voltage = 0;
        //     }
        // }
        
        previousTime = Timer.getFPGATimestamp();
        previousError = RPMError;

        endVoltage = voltage;
        Shooter.flywheel.setVolt(voltage);

        SmartDashboard.putNumber("VOLTAGE", voltage);
        SmartDashboard.putNumber("RPM", Shooter.flywheel.getRPM());
        SmartDashboard.putNumber("DESIRED RPM", desiredRPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Moving to ControlHopper. Running at " + Shooter.flywheel.getRPM() + " RPM");
        hasSwitched = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return /* Shooter.flywheel.getRPM() >= desiredRPM || */stopButton.get() || quitRamp;
    }

}
