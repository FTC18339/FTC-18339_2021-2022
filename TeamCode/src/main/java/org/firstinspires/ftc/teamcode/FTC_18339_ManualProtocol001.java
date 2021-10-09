package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC_18339_ManualProtocol001 extends Main {

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
                SetArmForces();

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

        if(gamepad1.y) math.SetMultiplier(0.3f);
        if(gamepad1.b) math.SetMultiplier(0.1f);
        //if(gamepad1.left_stick_button) math.SetDirection((float)Math.PI * 3 / 2);
        //if(gamepad1.right_stick_button) math.SetDirection((float)Math.PI / 2);
        //if(gamepad1.left_stick_button) dir = 1;
        //if(gamepad1.right_stick_button) dir = -1;
        double time = System.currentTimeMillis();

        //Algorithm determined wheel forces with the inputs
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        //Right trigger right rotation
        //Right trigger is right rotation, left trigger is left. Can slow rotations by depressing trigger less.
        float rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        int quad = math.GetQuad(x, y);
        double theta = math.Theta(x, y, quad);
        double z = (double)Math.sqrt(x * x + y * y);

        left_front_power = math.GetWheelForce(x, y, 1, rAxis, theta, z);
        left_back_power = math.GetWheelForce(x, y, 2, rAxis, theta, z);
        right_front_power = math.GetWheelForce(x, y, 3, rAxis, theta, z);
        right_back_power = math.GetWheelForce(x, y, 4, rAxis, theta, z);

        telemetry.addData("lpb", left_back_power + " q: " + quad + " theta: " + theta + "x: " + x + "y: " + y + "x2: " + gamepad1.right_stick_x);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
    }

    double hand_pos = 0f;
    public void SetArmForces() {
        if(!NoNullArmature()) return;

        double arm_power = 0f;

        //Set Arm and Hand movement

        if(gamepad1.dpad_left)
            arm_power = 0.15f;
        else if(gamepad1.dpad_right)
            arm_power = -0.15f;

        if(gamepad1.dpad_down)
            hand_pos = 1f;
        else if(gamepad1.dpad_up)
            hand_pos = 0f;

        arm.setVelocity(arm_power * MAX_NUM_TICKS_ARM * ARM_RPM);
        hand.setPosition(hand_pos);
        hand2.setPosition(hand_pos);
    }
}


