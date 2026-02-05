package org.firstinspires.ftc.teamcode.Auto.Paths;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.timeOutShooting;
import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.timeOutbetweenShoots;
import static org.firstinspires.ftc.teamcode.Constants.transferConstants.transferMotorPower;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Command.waitCommand;

@Autonomous
public class threeArtifactAuto_DriveAway extends autoRobotContainer {

    @Override
    public void path() {
        runShooter();

        schedule(
                new SequentialCommandGroup(
                        new MoveRobotEncoderXY_CMD(-22, -22, 3, 0.4, driveSub),
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower, timeOutShooting,false),
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower, timeOutShooting,false),
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, llSub, transferMotorPower, timeOutShooting,false)
                )
        );
    }
}
