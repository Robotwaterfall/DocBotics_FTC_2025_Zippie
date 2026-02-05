package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.llLockOnKp;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.tyOffSet;
import static org.firstinspires.ftc.teamcode.Constants.mecanumConstants.clippedRotLockOnPower;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

import java.util.function.Supplier;

public class teleOpMecanumDriveCommand extends CommandBase {


    private final mecanumDriveSubsystem driveSub;
    private final limelightSubsystem llSub;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rSupplier;
    private final Supplier<Double> tSupplier;
    private final Supplier<Boolean> resetSupplier;

    public teleOpMecanumDriveCommand(
            mecanumDriveSubsystem driveSub,
            limelightSubsystem llSub,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier,
            Supplier<Double> tSupplier,
            Supplier<Boolean> resetSupplier
    ) {

        this.driveSub = driveSub;
        this.llSub = llSub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.tSupplier = tSupplier;
        this.resetSupplier = resetSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute() {

        if(llSub.hasTarget() && tSupplier.get() > 0.5) {
            double forward = -ySupplier.get();
            double strafe = -xSupplier.get();

            double error = -(llSub.getTy() - tyOffSet);

            double rotPower = error * llLockOnKp;

            rotPower = Math.max(Math.min(rotPower, clippedRotLockOnPower), -clippedRotLockOnPower);

            driveSub.drive(forward, strafe, rotPower);

        }else{
            double forward = -ySupplier.get();
            double strafe = -xSupplier.get();
            double rotation = -rSupplier.get();

            driveSub.drive(forward, strafe, rotation);
        }

        boolean reset = resetSupplier.get();

        if(reset){
            driveSub.resetIMU();
        }
    }

}