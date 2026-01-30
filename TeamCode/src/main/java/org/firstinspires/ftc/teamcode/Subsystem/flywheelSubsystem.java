package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.tree.DCTree;

public class flywheelSubsystem extends SubsystemBase {

    DcMotor flyWheelMotor;

    public flywheelSubsystem(DcMotor flyWheelMotor){
        this.flyWheelMotor = flyWheelMotor;

        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public DcMotor getFlyWheelMotor(){


        return flyWheelMotor;
    }
}
