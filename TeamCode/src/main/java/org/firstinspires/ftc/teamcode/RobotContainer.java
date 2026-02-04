package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.cycleShoot;
import static org.firstinspires.ftc.teamcode.Constants.farZoneShooting;
import static org.firstinspires.ftc.teamcode.Constants.shooterPower;
import static org.firstinspires.ftc.teamcode.Constants.timeOutShooting;
import static org.firstinspires.ftc.teamcode.Constants.timeOutbetweenShoots;
import static org.firstinspires.ftc.teamcode.Constants.transferConstants.transferMotorPower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelAdjustCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Command.waitCommand;
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
                hardwareMap.get(DcMotor.class,"shooterMotor")
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



        Trigger transferTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7;
        } );

        Trigger outTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7;
        } );

        outTrigger.whileActiveContinuous(new teleOpTransferCommand(transferSub, -transferMotorPower, 100));

        transferTrigger.whileActiveContinuous(new SequentialCommandGroup(
                new teleOpTransferCommand(transferSub, transferMotorPower,timeOutShooting),
                new waitCommand(timeOutbetweenShoots) //seconds
        ));

        driverJoystick.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(new teleOpFlywheelCommand(flywheelSub, shooterPower));

        driverJoystick.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(new teleOpFlywheelCommand(flywheelSub, farZoneShooting));

        driverJoystick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileActiveContinuous(new SequentialCommandGroup(
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, transferMotorPower, timeOutShooting)

                ));

        driverJoystick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileActiveContinuous(new teleOpFlywheelCommand(flywheelSub, cycleShoot));

        driverJoystick.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new teleOpFlywheelAdjustCommand(llSub, flywheelSub));

    }

    private void runCommands() {
        // Add other commands here if needed


    }
}
