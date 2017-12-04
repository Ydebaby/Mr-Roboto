package testies;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.math.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
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
	
	static Pose ThePoseLol;
	
	
	static int fx = 200;
	static int fy = 0;

	
	static int state = 0;
		
	static Point polarHeading(int heading, Pose thePose)
	{
		float fucklort = Float.parseFloat(String.valueOf(Math.toRadians(thePose.getHeading()+heading)));
		float x = Float.parseFloat(String.valueOf(Math.cos(fucklort)));
		float y = Float.parseFloat(String.valueOf(Math.sin(fucklort)));
		Point polarPoint = new Point(x,y);
		return polarPoint;
	}
		
	static FileWriter logWrite()
	{
		//Date date = new Date();
		String filename = "File.txt";
		
		try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename, true)))
		{
			//PrintWriter printWriter = new PrintWriter(writer);
			
			//printWriter.println(new Timestamp(date.getTime()));
			writer.write("State: " + state + " ");
			//writer.write("Sensor side: " + senside*100 + "cm");
			//writer.write("Sensor front: " + senfront*100 + "cm");
			writer.write("Pose: " + ThePoseLol.getX() + " " + ThePoseLol.getY() + " " + ThePoseLol.getHeading() + '\n');
			
		} catch (Exception e)
		{
			e.printStackTrace();
		}
		
		return null;

		/*Date date = new Date();
		try(	FileWriter fileWriter = new FileWriter("File.txt", true);
				BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
				PrintWriter printWriter = new PrintWriter(bufferedWriter);	) 
		{
			printWriter.println(new Timestamp(date.getTime()));
			printWriter.println("State: " + state);
			printWriter.println("Pose: " + ThePoseLol.getX() + " " + ThePoseLol.getY() + " " + ThePoseLol.getHeading());

			
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
			// TODO: handle exception
		}
			return null;*/
		 	
	}
	
	public static void main (String[] args) throws InterruptedException
	{
		
		
		while(true)
		{	
			
			switch (state)
			{
				case 0: LCD.clear(); LCD.drawString("case 0", 1, 1);
			
					/*ThePoseLol = Poseo.getPose();
					logWrite();*/
					naviBot.addWaypoint(fx,fy);
					naviBot.followPath();
				
					distancefront.fetchSample(sampledistfront,0);
					distanceside.fetchSample(sampledistside,0);
					
					if(sampledistfront[0]<0.2)
					{
						//naviBot.stop();
						//ThePoseLol = Poseo.getPose();
						//logWrite();
						naviBot.clearPath();
						state = 1;
					}
					
					if(sampledistside[0]<0.3)
					{
						//naviBot.stop();
						//ThePoseLol = Poseo.getPose();
						//logWrite();
						naviBot.clearPath();
						state = 2;
					}
					
					break;
					
				case 1: LCD.clear(); LCD.drawString("case 1", 1, 1);

					//ThePoseLol = Poseo.getPose();
					//logWrite();
					Thread.sleep(100);
					//naviBot.clearPath();
					Point Left = polarHeading(-90,Poseo.getPose());
					naviBot.addWaypoint(Poseo.getPose().getX()+Left.x,Poseo.getPose().getY()+Left.y);
					naviBot.followPath();
					if(naviBot.waitForStop())
					state = 3;
					
					/*{
						distanceside.fetchSample(sampledistside,0);
						if(sampledistside[0]<0.3)
						{
							state = 2;
						}else if(sampledistside[0]>0.3) {
							state = 0;
						}
						
					}
					*/
					break;
					
					
				case 2: LCD.clear(); LCD.drawString("case 2", 1, 1);

						//ThePoseLol = Poseo.getPose();
						//logWrite();
						naviBot.clearPath();
						Thread.sleep(100);
						naviBot.addWaypoint(Poseo.getPose().getX()+5f,Poseo.getPose().getY());
						naviBot.followPath();
						if(naviBot.waitForStop())
						{
							distanceside.fetchSample(sampledistside,0);
							if(sampledistside[0]<0.3)
							{
								state = 3;
							}else if(sampledistside[0]>0.3)
							
							{
								/*naviBot.clearPath();
								naviBot.addWaypoint(fy, fx);
								naviBot.followPath();*/
								fx = 0;
								fy = 200;
								state = 0;
							}
							
						}
				
					break;
					
				case 3: LCD.clear(); LCD.drawString("case 3", 1, 1);
					Thread.sleep(100);
					distancefront.fetchSample(sampledistfront,0);
					distanceside.fetchSample(sampledistside,0);
					
//					if(sampledistfront[0]<0.2)
//					{
//						//naviBot.stop();
//						/*ThePoseLol = Poseo.getPose();
//						logWrite();*/
//						naviBot.clearPath();
//						state = 1;
//					}
					
					if(sampledistside[0]<0.3)
					{
						//naviBot.stop();
						//ThePoseLol = Poseo.getPose();
						//logWrite();
						naviBot.clearPath();
						state = 2;
					}
					
					//state = 0;
					
					break;
					
				default:
					
					break;
					
			}
		}

		
		/*while(true)
		{	
			
			
			LCD.clear(); LCD.drawString("start", 1, 1);
			distancefront.fetchSample(sampledistfront,0);
			distanceside.fetchSample(sampledistside,0);
			
			if(sampledistfront[0]<0.2)
			{
				naviBot.stop();
				naviBot.clearPath();
				state = 1;
			}
			
			if(sampledistside[0]<0.3)
			{
				naviBot.stop();
				naviBot.clearPath();
				state = 2;
			}
			
			naviBot.addWaypoint(fx,fy);
			naviBot.followPath();
			
			switch (state)
			{
				case 1: LCD.clear(); LCD.drawString("case 1", 1, 1);
					
					//naviBot.clearPath();
					Point Left = polarHeading(-90,Poseo.getPose());
					Thread.sleep(200);
					naviBot.addWaypoint(Poseo.getPose().getX()+Left.x,Poseo.getPose().getY()+Left.y);
					naviBot.followPath();
					if(naviBot.waitForStop())
					{
					state = 0;
					}
					break;
					
					
				case 2: LCD.clear(); LCD.drawString("case 2", 1, 1);
					
					//naviBot.clearPath();
					naviBot.addWaypoint(Poseo.getPose().getX()+5f,Poseo.getPose().getY());
					naviBot.followPath();
					if(naviBot.waitForStop())
					{
						state = 0;
					}
					break;
					
				default:
					
					break;
					
			}
		}*/
	}
}
