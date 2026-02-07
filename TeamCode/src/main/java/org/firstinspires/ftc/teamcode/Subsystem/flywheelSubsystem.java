package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class flywheelSubsystem extends SubsystemBase {

    DcMotorEx flyWheelMotor;

    public flywheelSubsystem(DcMotorEx flyWheelMotor){
        this.flyWheelMotor = flyWheelMotor;

        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public DcMotorEx getFlyWheelMotor(){


        return flyWheelMotor;
    }

    public void setVelocity(double velocity){
        flyWheelMotor.setVelocity(velocity, AngleUnit.RADIANS);
    }

}
