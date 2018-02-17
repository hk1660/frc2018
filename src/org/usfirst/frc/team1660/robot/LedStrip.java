package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedStrip {

	private static DigitalOutput pinA;
	private static DigitalOutput pinB;
	private static DigitalOutput pinColor;
	private static String allianceColor;
	private static DriverStation ds;
	
	public LedStrip(){
		
		ds = DriverStation.getInstance();
		
		pinA = new DigitalOutput(RobotMap.PIN_A_PORT);
		pinB = new DigitalOutput(RobotMap.PIN_B_PORT);
		pinColor = new DigitalOutput(RobotMap.PIN_COLOR_PORT);
		
		if(ds.getAlliance() == DriverStation.Alliance.Red){
			allianceColor = "red";
		} else if(ds.getAlliance() == DriverStation.Alliance.Blue){
			allianceColor = "blue";
		} else {
			allianceColor = "unknown";
		}
		
		SmartDashboard.putString("AllianceColor", allianceColor);
		
		ledTurnOff();
		
	}
	
	
	public static void ledLockTheClimbNow(){		
		pinA.set(true);
		pinB.set(true);
	}
	
	
	public static void ledAllianceColor(){
		if(allianceColor == "red"){
			ledAllianceColorRed();
		} else if (allianceColor == "blue"){
			ledAllianceColorBlue();
		} else {
			ledTurnOff();
		}
	}
	
	public static void ledAllianceColorBlue(){
		pinA.set(true);
		pinB.set(false);
		pinColor.set(true);
	}

	public static void ledAllianceColorRed(){
		pinA.set(true);
		pinB.set(false);
		pinColor.set(false);
	}

	public static void ledTurnOff(){
		pinA.set(false);
		pinB.set(false);
	}
	
	public static void partyTime(){
		pinA.set(false);
		pinB.set(true);
	}
	
	public static String getAllianceColor(){
		return LedStrip.allianceColor;
	}

	
}
