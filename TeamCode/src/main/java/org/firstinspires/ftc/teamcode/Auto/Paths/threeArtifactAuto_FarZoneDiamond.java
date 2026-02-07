package org.firstinspires.ftc.teamcode.Auto.Paths;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.farZoneShootingVelocity;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.timeOutShooting;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.timeOutbetweenShots;
import static org.firstinspires.ftc.teamcode.Constants.transferConstants.transferMotorPower;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Command.autoLimelightLockCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Command.waitCommand;

@Autonomous
public class threeArtifactAuto_FarZoneDiamond extends autoRobotContainer {


    @Override
    public void path() {



        schedule(

                new SequentialCommandGroup(
                        new MoveRobotEncoderXY_CMD(3.91,3.91,4,0.35, driveSub),

                        new waitCommand(2.5),

                        new autoLimelightLockCommand(llSub,driveSub),

                        new teleOpFlywheelCommand(flywheelSub, farZoneShootingVelocity),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower,
                                timeOutShooting, true),
                        new waitCommand(timeOutbetweenShots),

                        new teleOpFlywheelCommand(flywheelSub, farZoneShootingVelocity),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower,
                                timeOutShooting, true),
                        new waitCommand(timeOutbetweenShots),

                        new teleOpFlywheelCommand(flywheelSub, farZoneShootingVelocity),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower,
                                timeOutShooting, true),
                        new waitCommand(timeOutbetweenShots)


                )
        );


    }
}
