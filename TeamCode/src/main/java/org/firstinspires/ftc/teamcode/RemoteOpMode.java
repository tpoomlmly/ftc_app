package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Remote operation", group = "Competition")
public class RemoteOpMode extends LinearOpMode {

    // Team no. 5
    @Override
    public void runOpMode(){
        HardwareInitialiser h = new HardwareInitialiser(hardwareMap);

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        h.runtime.reset();

        double powerL;
        double powerR;
        double speed;
        double direction;
        int rotorDirection;
        while (opModeIsActive()) {
            telemetry.addData("Running time", h.runtime.time());

            speed = -gamepad1.left_stick_y;
            direction = 2*gamepad1.left_trigger - 1; //On the REV gamepad the right stick x is mapped as the left trigger
            //speed = gamepad1.right_trigger - gamepad1.left_trigger;
            //direction = gamepad1.right_stick_x;
            rotorDirection = (gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0);

            powerL = Range.clip(speed + direction, -1.0, 1.0) ;
            powerR = Range.clip(speed - direction, -1.0, 1.0) ;

            h.leftMotor.setPower(powerL);
            h.rightMotor.setPower(powerR);
            h.rotor.setPower(rotorDirection);

            telemetry.addData("Left motor power", h.leftMotor.getPower());
            telemetry.addData("Right motor power", h.rightMotor.getPower());
            telemetry.update();
        }
    }
}