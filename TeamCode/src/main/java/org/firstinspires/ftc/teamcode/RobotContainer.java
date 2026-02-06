package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.timeOutShooting;
import static org.firstinspires.ftc.teamcode.Constants.transferConstants.transferMotorPower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Command.spinFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.transferSubsystem;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private mecanumDriveSubsystem driveSub;
    private flywheelSubsystem flywheelSub;
    private transferSubsystem transferSub;
    private limelightSubsystem llSub;
    private GamepadEx driverJoystick;

    @Override
    public void initialize() {
        driveSub = new mecanumDriveSubsystem(
                hardwareMap.get(DcMotor.class,"front_left"),
                hardwareMap.get(DcMotor.class, "front_right"),
                hardwareMap.get(DcMotor.class, "back_left"),
                hardwareMap.get(DcMotor.class, "back_right"),
                hardwareMap
        );

        flywheelSub = new flywheelSubsystem(
                hardwareMap.get(DcMotorEx.class,"shooterMotor")
        );

        transferSub = new transferSubsystem(
                hardwareMap.get(DcMotor.class,"transferMotor")
        );

        llSub = new limelightSubsystem(
                hardwareMap
        );

        driverJoystick = new GamepadEx(gamepad1);

        setDefaultCommands();
        runCommands();


    }

    /**
     * Apply a joystick deadband so tiny inputs don’t move the motors.
     *
     * @param value     joystick value
     * @param threshold minimum absolute value to count as input
     * @return filtered value
     */
    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) > threshold) ? value : 0.0;
    }

    public void setDefaultCommands() {

        driveSub.setDefaultCommand(
                new teleOpMecanumDriveCommand(
                        driveSub,
                        llSub,
                        () -> applyDeadband(driverJoystick.getLeftY(), 0.05),  // Forward/back
                        () -> applyDeadband(driverJoystick.getLeftX(), 0.05),  // Strafe
                        () -> applyDeadband(driverJoystick.getRightX(), 0.05), // Rotate
                        () -> driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        () -> driverJoystick.getButton(GamepadKeys.Button.START)        // Reset
                )
        );



        Trigger startShooterSequanceTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7;
        } );


        Trigger outTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7;
        } );

        outTrigger.whileActiveContinuous(new teleOpTransferCommand(transferSub, -transferMotorPower, 1000));



        startShooterSequanceTrigger.whileActiveContinuous(
               new SequentialCommandGroup(
                       new spinFlywheelCommand(flywheelSub, llSub),
                       new teleOpTransferCommand(transferSub, transferMotorPower, timeOutShooting)
               )
        );





    }

    private void runCommands() {
        // Add other commands here if needed


    }
}
