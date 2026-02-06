package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;

public class moveForward extends autoRobotContainer {

    @Override
    public void path() {
        schedule(
                new SequentialCommandGroup(
                        new MoveRobotEncoderXY_CMD(5,5, 4, 0.3, driveSub)
                )
        );
    }
}
