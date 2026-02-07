package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterTolerance;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;

public class teleOpFlywheelCommand extends CommandBase {

    flywheelSubsystem flywheelSub;
    DcMotorEx flyWheelMotor;
    double flyWheelPower;

    public teleOpFlywheelCommand(flywheelSubsystem flywheelSub, double flywheelPower){
        this.flywheelSub = flywheelSub;
        this.flyWheelMotor = flywheelSub.getFlyWheelMotor();
        this.flyWheelPower = flywheelPower;
        addRequirements(flywheelSub);

    }



    @Override
    public void execute() {

        flywheelSub.setVelocity(flyWheelPower);

    }

    @Override
    public boolean isFinished() {
        boolean isAtSetVelocity = (flyWheelMotor.getVelocity(AngleUnit.RADIANS) > (flyWheelPower - shooterTolerance)) &&
                (flyWheelMotor.getVelocity(AngleUnit.RADIANS) < (flyWheelPower + shooterTolerance));
        return isAtSetVelocity;
    }


}
