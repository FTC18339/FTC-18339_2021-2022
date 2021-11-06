package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC_18339_TankProtocol001 extends Main {
    private double left_front_power = 0;
    private double left_back_power = 0;
    private double right_front_power = 0;
    private double right_back_power = 0;

    private double rollerTime = 0f;
    private double conveyorTime = 0f;

    private double rollerMultiplier = 0.5f;
    private double otherSpeedMultiplier = 1f;
    private double moveMultiplier = 1f;

    int dir = 1;
    double adder = 0;

    @Override
    public void runOpMode() {

        //Initialize all hardware, servos, and sensors, as well as the algorithm
        initMaths();
        initHardware();
        initManualModes();

        waitForStart();

        //initColorSensor();

        if (opModeIsActive()) {
            // Loops over time to check for inputs and to update motors
            //math.SetDirection(3 / 2);

            while (opModeIsActive()) {
                // Show motor info on android phone

                //Runtime Loop to update hardware with input, and automation
                SetMotorForces();
                SetSpinnerForces();
                SetCraneForces();

                //Update phone info
                telemetry.update();
                idle();
            }
        }
    }

    boolean wasMovingLastFrame = false;
    boolean notRunningConveyor = true;
    boolean notRunningRoller = true;
    public void SetMotorForces() {
        if(!NoNullHardware()) return;

        //if(gamepad1.y) math.SetMultiplier(0.3f);
        //if(gamepad1.b) math.SetMultiplier(0.05f);
        //if(gamepad1.left_stick_button) math.SetDirection((float)Math.PI * 3 / 2);
        //if(gamepad1.right_stick_button) math.SetDirection((float)Math.PI / 2);
        //if(gamepad1.left_stick_button) dir = 1;
        //if(gamepad1.right_stick_button) dir = -1;
        double time = System.currentTimeMillis();

        //Algorithm determined wheel forces with the inputs
        double y = gamepad1.left_stick_y;

        //Right trigger right rotation
        //Right trigger is right rotation, left trigger is left. Can slow rotations by depressing trigger less.
        float rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        left_front_power = -math.GetWheelForceTank(y, 1, rAxis);
        left_back_power = math.GetWheelForceTank(y, 2, rAxis);
        right_front_power = -math.GetWheelForceTank(y, 3, rAxis);
        right_back_power = math.GetWheelForceTank(y, 4, rAxis);

        telemetry.addData("lpb", left_back_power);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
    }

    public void SetSpinnerForces() {
        if(!NoNullSpinner()) return;

        boolean x = gamepad1.x;
        boolean b = gamepad1.b;

        if(x) {
            telemetry.addData("x","x is active");
            spinner.setPower(1);
        } else if(b) {
            spinner.setPower(-1);
        } else {
            spinner.setPower(0);
        }
    }

    public void SetCraneForces()
    {
        //Rotation of the entire arm itself, right trigger is right rotation, left trigger is left
        double r = gamepad2.right_trigger - gamepad2.left_trigger;
        //Rotation of the first arm joint at the base of the arm.
        double r1 = gamepad2.left_stick_y;
        //Rotation of the second arm joint.
        double r2 = gamepad2.right_stick_y;

        
    }
}
