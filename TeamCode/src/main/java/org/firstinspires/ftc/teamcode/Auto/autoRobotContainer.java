package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelAdjustCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpFlywheelCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.flywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.transferSubsystem;

public class autoRobotContainer extends CommandOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public mecanumDriveSubsystem driveSub;
    public limelightSubsystem llSub;

    public DcMotor shooterMotor;
    public flywheelSubsystem flywheelSub;
    public DcMotor transferMotor;
    public transferSubsystem transferSub;






    @Override
    public void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");


        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        initSubsystems();
        path();


    }


    private void initSubsystems(){
        driveSub = new mecanumDriveSubsystem(frontLeft, frontRight, backLeft, backRight, hardwareMap);

        flywheelSub = new flywheelSubsystem(shooterMotor);

        transferSub = new transferSubsystem(transferMotor);

        llSub = new limelightSubsystem(hardwareMap);


    }

    public void runShooter(){
        schedule(
                new SequentialCommandGroup(
                        new teleOpFlywheelCommand(flywheelSub, shooterPower)
                )
        );
    }

    public void runLimelightShooter(){
        schedule(
                new SequentialCommandGroup(
                        new teleOpFlywheelAdjustCommand(llSub, flywheelSub)
                )
        );
    }



    public void stopShooter(){
        schedule(
                new SequentialCommandGroup(
                        new teleOpFlywheelCommand(flywheelSub, 0)
                )
        );
    }


    public void path(){

    }
}
