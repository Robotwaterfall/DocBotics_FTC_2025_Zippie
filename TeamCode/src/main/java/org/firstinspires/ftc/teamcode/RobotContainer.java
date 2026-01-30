package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.transferSubsystem;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private mecanumDriveSubsystem driveSub;
    private flywheelSubsystem flywheelSub;
    private transferSubsystem transferSub;
    private GamepadEx driverJoystick;

    @Override
    public void initialize() {

        // Mecanum Motor binding
        driveSub = new mecanumDriveSubsystem(
                hardwareMap.get(DcMotor.class,"front_left"),
                hardwareMap.get(DcMotor.class, "front_right"),
                hardwareMap.get(DcMotor.class, "back_left"),
                hardwareMap.get(DcMotor.class, "back_right"),
                hardwareMap
        );

        driverJoystick = new GamepadEx(gamepad1);

        runCommands();
        setDefaultCommands();

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
        Trigger transferTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7;
        } );

        Trigger shooterTrigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7;
        } );

        shooterTrigger.whileActiveContinuous(new teleOpFlywheelCommand(flywheelSub));

        transferTrigger.whileActiveContinuous(new teleOpTransferCommand(transferSub));
    }

    private void runCommands() {
        // Add other commands here if needed


    }
}
