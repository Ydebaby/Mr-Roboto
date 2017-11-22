package testies;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.math.*;
import java.util.Date;
import java.sql.Timestamp;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.*;
import lejos.robotics.geometry.Point;
import lejos.robotics.navigation.*;
import lejos.robotics.localization.*;
import lejos.robotics.localization.CompassPoseProvider.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;
import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;

import jdk.jfr.events.FileWriteEvent;

public class stateofthemachine
{
	//Init
	static RegulatedMotor leftm = new EV3LargeRegulatedMotor(MotorPort.A); //motor 1
	static RegulatedMotor rightm = new EV3LargeRegulatedMotor(MotorPort.B); //motor 2
	
	static Wheel wheelLeft = WheeledChassis.modelWheel(leftm,4.32).offset(-6.35);
	static Wheel wheelRight = WheeledChassis.modelWheel(rightm,4.32).offset(6.35);
	static Chassis chassis = new WheeledChassis(new Wheel[]{wheelLeft, wheelRight},WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot Bronchio = new MovePilot(chassis);
	static OdometryPoseProvider Poseo = new OdometryPoseProvider(Bronchio);
	static Navigator naviBot = new Navigator(Bronchio, Poseo); 
	
	
	static Port port2 = LocalEV3.get().getPort("S2"); //ultrasonic
	static SensorModes sens1UltraFront = new EV3UltrasonicSensor(port2);
	
	static Port port3 = LocalEV3.get().getPort("S3"); //ultrasonic
	static SensorModes sens2UltraSide = new EV3UltrasonicSensor(port3);
	
	static SampleProvider distancefront = sens1UltraFront.getMode("Distance");
	static float[] sampledistfront = new float[distancefront.sampleSize()];
	
	static SampleProvider distanceside = sens2UltraSide.getMode("Distance");
	static float[] sampledistside = new float[distanceside.sampleSize()];
	
	static int fx = 200;
	static int fy = 0;
	
	static Pose start = new Pose();
	static Pose newpose = new Pose();
	
	static int state = 1;
		
	static Point polarHeading(int heading, Pose thePose)
	{
		float fucklort = Float.parseFloat(String.valueOf(Math.toRadians(thePose.getHeading()+heading)));
		float x = Float.parseFloat(String.valueOf(Math.cos(fucklort)));
		float y = Float.parseFloat(String.valueOf(Math.sin(fucklort)));
		Point polarPoint = new Point(x,y);
		return polarPoint;
	}
		
	static PrintWriter logWrite()
	{
		Date date = new Date();
		try(	FileWriter fileWriter = new FileWriter("File.txt", true);
				BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
				PrintWriter printWriter = new PrintWriter(bufferedWriter);	) 
		{
			printWriter.println("State: " + state);
			printWriter.println(new Timestamp(date.getTime()));
			
		} catch (IOException e)
		{
			e.printStackTrace();
		}
		    
			return null;
		
		//The example below creates a new textfile every time
		/*try
		{
			File file = new File ("/home/lejos/programs/text.txt");
		    PrintWriter printWriter = new PrintWriter (file);
		    printWriter.println (state);
		    printWriter.close ();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
			return null;*/
		 	
	}
	
	public static void main (String[] args) throws InterruptedException
	{
		naviBot.addWaypoint(fx,fy);
		naviBot.followPath();
		
		while(true)
		{	
			switch (state)
			{
				case 1: LCD.clear(); LCD.drawString("case 1", 1, 1);
					
					naviBot.followPath();
					distancefront.fetchSample(sampledistfront,0);
					Thread.sleep(100);
					if(sampledistfront[0]<0.2)
					{
						naviBot.clearPath();
						logWrite();
						state = 2;
					}
					break;
					
				case 2: LCD.clear(); LCD.drawString("case 2", 1, 1);
					
					naviBot.clearPath();
					logWrite();
					newpose = Poseo.getPose();
					Point turnLeft = polarHeading(-90,newpose);
					naviBot.addWaypoint(newpose.getX()+1f*turnLeft.x,newpose.getY()+1f*turnLeft.y);
					naviBot.followPath();
					Thread.sleep(500);
					
					state = 4;
					break;
				/*case 3: LCD.clear(); LCD.drawString("Case 3", 1, 1);
				
					naviBot.clearPath();
					newpose = Poseo.getPose();
					Point turnRight = polarHeading(90,newpose);
					naviBot.addWaypoint(newpose.getX()+1f*turnRight.x,newpose.getY()+1f*turnRight.y);
					Thread.sleep(500);
					naviBot.followPath();
					state = 4;
					break;*/
				case 4: LCD.clear(); LCD.drawString("Case 4", 1, 1);
				
					naviBot.clearPath();
					logWrite();
					distanceside.fetchSample(sampledistside, 0);
					
					while(sampledistside[0]<0.2)
					{
						newpose = Poseo.getPose();
						Thread.sleep(100);
						naviBot.addWaypoint(newpose.getX()+5f,newpose.getY(),newpose.getHeading());
						naviBot.followPath();
						distanceside.fetchSample(sampledistside, 0);
						
					}
					state = 1;
					break;
					
			}
		}
	}
}
