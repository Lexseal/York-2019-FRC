package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RecordingReader;

public class AutoSwitchFromMiddle extends AutoMode {

	public AutoSwitchFromMiddle(Drive _drive, Elevator _lifter, Intake _intake, double _freq) {
		super(_drive, _lifter, _intake, _freq);
		
		reader.add(new RecordingReader("M_LSwitch"));
		reader.add(new RecordingReader("M_RSwitch"));
	}
	
	public void initialize(int[] plateAssignment) {
		System.out.println("switchFromMiddle");
		
		if (!drive.isAlive()) {
			drive.start();
		}
		drive.zeroSensor();
		
		if (plateAssignment[0] == -1) { //If scale on the left, remove right auto.
			reader.remove(1);
			System.out.println("rightAutoRemoved");
		} else if (plateAssignment[0] == 1) { //If scale on the right, remove left auto.
			reader.remove(0);
			System.out.println("leftAutoRemoved");
		} else { //If info inconclusive, remove both.
			reader.remove(0);
			reader.remove(0);
			System.out.println("bothAutoRemoved");
		}
	}

}
