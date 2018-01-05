package testies;


import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.math.*;
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
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Point;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.*;
import lejos.robotics.localization.*;
import lejos.robotics.localization.CompassPoseProvider.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class Speedy
{
	static RegulatedMotor leftm = new EV3LargeRegulatedMotor(MotorPort.A); //motor 1
	static RegulatedMotor rightm = new EV3LargeRegulatedMotor(MotorPort.B); //motor 2
	
	static Wheel wheelLeft = WheeledChassis.modelWheel(leftm,4.32).offset(-6.35);
	static Wheel wheelRight = WheeledChassis.modelWheel(rightm,4.32).offset(6.35);
	static Chassis chassis = new WheeledChassis(new Wheel[]{wheelLeft, wheelRight},WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot Bronchio = new MovePilot(chassis);
	static OdometryPoseProvider Poseo = new OdometryPoseProvider(Bronchio);
	static Navigator naviBot = new Navigator(Bronchio, Poseo); 
	
	static Pose start = new Pose();
	//start = Poseo.getPose();
	static Pose newpose = new Pose();
	
	static Port port2 = LocalEV3.get().getPort("S2"); //ultrasonic
	static SensorModes sens1UltraFront = new EV3UltrasonicSensor(port2);
	
	static int fx = 200;
	static int fy = 0;
	
	static SampleProvider distancefront = sens1UltraFront.getMode("Distance");
	static float[] sampledistfront = new float[distancefront.sampleSize()];
	
	/*SampleProvider distanceside = sens2UltraSide.getMode("Distance");
	float[] sampledistside = new float[distanceside.sampleSize()];*/
	
	/*static Port port3 = LocalEV3.get().getPort("S3"); //ultrasonic
	static SensorModes sens2UltraSide = new EV3UltrasonicSensor(port3);*/
	
	static Pose ThePoseLol;
	static FileWriter logWrite()
	{
		//Date date = new Date();
		String filename = "File.txt";
		
		try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename, true)))
		{
			writer.write("Pose: " + ThePoseLol.getX() + " " + ThePoseLol.getY() + " " + ThePoseLol.getHeading() + '\n');
			
		} catch (Exception e)
		{
			e.printStackTrace();
		}
		
		return null;
	}
	
	static Point polarHeading(int heading, Pose thePose)
	{
		
		float fucklort = Float.parseFloat(String.valueOf(Math.toRadians(thePose.getHeading()+heading)));
		float x = Float.parseFloat(String.valueOf(Math.cos(fucklort)));
		float y = Float.parseFloat(String.valueOf(Math.sin(fucklort)));
		Point polarPoint = new Point(x,y);
		return polarPoint;
	}
	
	public static void main(String[] args) throws InterruptedException
	{
			ThePoseLol = Poseo.getPose();
			logWrite();
			//naviBot.singleStep(true);
			naviBot.addWaypoint(fx,fy);
			naviBot.followPath();
			
			while(naviBot.isMoving())
			{				
				naviBot.followPath();
				distancefront.fetchSample(sampledistfront,0);
				if(sampledistfront[0]<0.20)
				{
					ThePoseLol = Poseo.getPose();
					logWrite();
					//naviBot.stop(); 
					naviBot.clearPath(); // OG code
					//newpose = Poseo.getPose();
					
					Point Left = polarHeading(-90,Poseo.getPose()); // from statofthemachine
					//Point left = polarHeading(-90,newpose); //OG code
					//Point turnRight = polarHeading(90,newpose);
					
					
					naviBot.addWaypoint(Poseo.getPose().getX()+20f*Left.x,Poseo.getPose().getY()+20f*Left.y); // from statofthemachine
					//naviBot.addWaypoint(newpose.getX()+20f*left.x,newpose.getY()+20f*left.y);		//OG code
					naviBot.addWaypoint(/*start.getX()+*/fx,/*start.getY()+*/fy);
					
					naviBot.followPath(); //OG code
					//Poseo.equals(null);
					Thread.sleep(500);
					
					
					//Delay.msDelay(50000);
					
				}	
			}
			System.exit(0);	
	}

}
