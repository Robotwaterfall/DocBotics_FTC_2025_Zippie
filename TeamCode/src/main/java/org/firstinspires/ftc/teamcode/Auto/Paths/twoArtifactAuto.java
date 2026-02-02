package org.firstinspires.ftc.teamcode.Auto.Paths;

import static org.firstinspires.ftc.teamcode.Constants.shooterPower;
import static org.firstinspires.ftc.teamcode.Constants.timeOutShooting;
import static org.firstinspires.ftc.teamcode.Constants.timeOutbetweenShoots;
import static org.firstinspires.ftc.teamcode.Constants.transferConstants.transferMotorPower;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpTransferCommand;
import org.firstinspires.ftc.teamcode.Command.waitCommand;

@Autonomous
public class twoArtifactAuto extends autoRobotContainer {

    @Override
    public void path() {
        runShooter();

        schedule(
                new SequentialCommandGroup(
                        new MoveRobotEncoderXY_CMD(-22, -22, 3, 0.4, driveSub),
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, transferMotorPower, timeOutShooting),
                        new waitCommand(timeOutbetweenShoots),
                        new teleOpTransferCommand(transferSub, transferMotorPower, timeOutShooting)
                )
        );

    }

}
