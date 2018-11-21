package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RemoteOpMode extends LinearOpMode {

    private DcMotor motorL;
    private DcMotor motorR;

    @Override
    public void runOpMode(){
        motorL = hardwareMap.get(DcMotor.class, "motor 0 HD hex");
        motorR = hardwareMap.get(DcMotor.class, "motor 2 HD hex");

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();

        double powerL;
        double powerR;
        while (opModeIsActive()) {
            powerL = this.gamepad1.left_stick_y;
            powerR = this.gamepad1.right_stick_y;

            motorL.setPower(powerL);
            motorR.setPower(powerR);

            telemetry.addData("Left motor power", motorL.getPower());
            telemetry.addData("Right motor power", motorR.getPower());
            telemetry.update();
        }
    }
}
