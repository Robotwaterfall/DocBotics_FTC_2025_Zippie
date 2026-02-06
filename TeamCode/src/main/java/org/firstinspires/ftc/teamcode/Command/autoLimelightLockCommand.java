package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.ShooterLockedZone;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.tyOffSet;
import static org.firstinspires.ftc.teamcode.Constants.llLockOnKp;
import static org.firstinspires.ftc.teamcode.Constants.mecanumConstants.clippedRotLockOnPower;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class autoLimelightLockCommand extends CommandBase {
    limelightSubsystem llSub;
    mecanumDriveSubsystem driveSub;

    public autoLimelightLockCommand(limelightSubsystem llSub,
                                    mecanumDriveSubsystem driveSub){
        this.llSub = llSub;
        this.driveSub = driveSub;
        addRequirements(driveSub);

    }

    @Override
    public void initialize() {
        driveSub.drive(0,0,0);
    }

    @Override
    public void execute() {

        double error = -(llSub.getTy() - tyOffSet);

        double rotPower = error * llLockOnKp;

        rotPower = Math.max(Math.min(rotPower, clippedRotLockOnPower), -clippedRotLockOnPower);

        driveSub.drive(0,0, rotPower);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(llSub.getTy()) <= ShooterLockedZone;
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0,0,0);
    }
}
