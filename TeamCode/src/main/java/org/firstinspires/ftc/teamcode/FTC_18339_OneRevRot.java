package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FTC_18339_OneRevRot extends FTC_18339_AutonomousProtocol002 {
    @Override
    public void ChildCommandInitialization(){
        commands = new Command[] {
                new Command("ONEREVROT", 0, true, 0)
        };
    }
}
