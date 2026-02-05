package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.max_Shooter_Power;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.max_Tx;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.min_Shooter_Power;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.min_Tx;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;

public class teleOpFlywheelAdjustCommand extends CommandBase {
    limelightSubsystem llSub;
    flywheelSubsystem flywheelSub;
    DcMotor flywheel;


    public teleOpFlywheelAdjustCommand(limelightSubsystem llSub, flywheelSubsystem flywheelSub){
        this.llSub = llSub;
        this.flywheelSub = flywheelSub;
        this.flywheel = flywheelSub.getFlyWheelMotor();
        addRequirements(flywheelSub);

    }

    @Override
    public void initialize() {
        flywheel.setPower(0);
    }

    @Override
    public void execute() {
            // Since camera is flipped 90 degrees, tx now represents the vertical distance
            double rawDistanceAxis = llSub.getTx();

            // 1) Clamp the input to your known range
            // Note: Use your calibrated min/max constants here
            double clampedVal = Math.max(min_Tx, Math.min(max_Tx, rawDistanceAxis));

            // 2) Normalize to a 0.0 - 1.0 range
            double normalized = (clampedVal - min_Tx) / (max_Tx - min_Tx);

            // --- IMPORTANT: INVERSION CHECK ---
            // If you move further away and the motor slows down,
            // uncomment the line below:
            // normalized = 1.0 - normalized;


            // 3) Calculate Shooter Power
            // If being farther away (higher tx) needs MORE power:
            double shooterPower = min_Shooter_Power + (normalized * (max_Shooter_Power - min_Shooter_Power));

            // 4) Final Clamp to ensure we never exceed motor limits
            // Math.min(higherValue, Math.max(lowerValue, actualValue))
            double finalPower = Math.max(min_Shooter_Power, Math.min(max_Shooter_Power, shooterPower));

            flywheel.setPower(finalPower);



    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setPower(0);
    }
}
