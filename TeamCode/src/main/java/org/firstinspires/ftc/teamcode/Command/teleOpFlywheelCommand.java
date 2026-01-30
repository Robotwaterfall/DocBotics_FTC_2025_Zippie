package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;

public class teleOpFlywheelCommand extends CommandBase {

    flywheelSubsystem flywheelSub;
    DcMotor flyWheelMotor;

    public teleOpFlywheelCommand(flywheelSubsystem flywheelSub){
        this.flywheelSub = flywheelSub;
        this.flyWheelMotor = flywheelSub.getFlyWheelMotor();
        addRequirements(flywheelSub);

    }

    @Override
    public void initialize() {
        flyWheelMotor.setPower(0);

    }

    @Override
    public void execute() {

        flyWheelMotor.setPower(Constants.shooterConstants.shooterPower);

    }

    @Override
    public void end(boolean interrupted) {
        flyWheelMotor.setPower(0);

    }

}
