package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.lockOnTolerance;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.tyOffSet;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.G_inchPerSecondSquared;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.RIM_RADIUS_INCH;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.efficiency;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.h;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.shooterTolerence;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.theta;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.y0;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;

public class spinFlywheelCommand extends CommandBase {
    flywheelSubsystem flySub;
    limelightSubsystem llSub;
    DcMotorEx flyMotor;

    double desiredTangentialVelocity;

    public spinFlywheelCommand(flywheelSubsystem flySub, limelightSubsystem llSub){
        this.flySub = flySub;
        this.flyMotor = flySub.getFlyWheelMotor();

    }


    @Override
    public void execute() {

        double x = llSub.getDistanceInch();

        double thetaRad = Math.toRadians(theta);
        double X = x + RIM_RADIUS_INCH;
        double deltaH = h - y0;

//        if(Math.tan(thetaRad) <= (2 * deltaH) / X){
//            //shot is too flat; might need to increase angle or accept a rising shoot
//        }

        double numerator = G_inchPerSecondSquared * Math.pow(X, 2);
        double denominator = 2 * Math.pow(Math.cos(thetaRad), 2) * (X * Math.tan(thetaRad) - deltaH);

        if(denominator <= 0) {numerator = 0.0;} //Impossible shot

        double vExit = Math.sqrt(numerator / denominator);

        desiredTangentialVelocity = (vExit * 2.0) / efficiency;

        flySub.setMotorTangentialVelocity(desiredTangentialVelocity);


    }

    @Override
    public boolean isFinished() {
        boolean isFlywheelAtSpeed = (flySub.getMotorTangentialVelocity() > (desiredTangentialVelocity - shooterTolerence)) &&
                (flySub.getMotorTangentialVelocity() < (desiredTangentialVelocity + shooterTolerence));

        boolean isAligned = (llSub.getTx() > (tyOffSet - lockOnTolerance)) &&
                (flySub.getMotorTangentialVelocity() < (tyOffSet + lockOnTolerance));

        return isFlywheelAtSpeed && isAligned;
    }

}
