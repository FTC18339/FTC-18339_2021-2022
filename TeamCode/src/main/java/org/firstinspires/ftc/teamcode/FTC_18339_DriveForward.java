package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FTC_18339_DriveForward extends FTC_18339_AutonomousProtocol002 {
    @Override
    public void ChildCommandInitialization(){
        commands = new Command[] {
                new Command("MOVE", -50 * mmPerInch, true, 0),
        };
    }
}
