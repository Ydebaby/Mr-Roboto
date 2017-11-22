package testies;

import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;
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
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.chassis.*;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.*;
import lejos.robotics.localization.*;
import lejos.robotics.localization.CompassPoseProvider.*;
import lejos.robotics.navigation.Move;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class Testing2 {
	
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
	

	/*
	static Port port1 = LocalEV3.get().getPort("S1"); //compass
	static SensorModes sens1Comp = new HiTechnicCompass(port1);
	*/
	
	public static void main(String[] args) throws InterruptedException {
		
		
		
		SampleProvider distancefront = sens1UltraFront.getMode("Distance");
		float[] sampledistfront = new float[distancefront.sampleSize()];
		
		SampleProvider distanceside = sens2UltraSide.getMode("Distance");
		float[] sampledistside = new float[distanceside.sampleSize()];
		
		/*SampleProvider angle = sens1Comp.getMode("Compass");
		float[] sampledeg = new float[angle.sampleSize()];*/
		
		
		StringBuffer sb = new StringBuffer(16);
		
		
			
		
		while(true)
		{
			
			Pose start = new Pose();
			//start = Poseo.getPose();
			Pose newpose = new Pose();
			naviBot.addWaypoint(100,0);
			naviBot.followPath();
			distancefront.fetchSample(sampledistfront,0);
			if(sampledistfront[0]<0.2)
			{
				//naviBot.stop();
				naviBot.clearPath();
				newpose = Poseo.getPose();
				float fucklort = Float.parseFloat(String.valueOf(Math.toRadians(newpose.getHeading()-90)));
				float a = Float.parseFloat(String.valueOf(Math.cos(fucklort)));
				float b = Float.parseFloat(String.valueOf(Math.sin(fucklort)));
				
				naviBot.addWaypoint(newpose.getX()+40f*a,newpose.getY()+40f*b);
				//Delay.msDelay(100);
				naviBot.addWaypoint(start.getX()+100,start.getY());
				naviBot.followPath();
				while(naviBot.isMoving()) 
				{
					Thread.sleep(500);
				}
				if(naviBot.waitForStop())
				{
					System.exit(0);
				}
				//Delay.msDelay(50000);
				
				
			}
			
			//Button.ESCAPE.waitForPressAndRelease();
				
				
				
				
				
			
			
			
			/*
			distance.fetchSample(sampledist, 0);
			leftm.forward();
			rightm.forward();
			if(sampledist[0]<0.3)
			{
				leftm.forward();
				rightm.backward();
				Delay.msDelay(700); // ~ 90 degrees at this speed
			}
			*/
			
			//naviBot.getPoseProvider().getPose();
			
			
			/*//current best
			naviBot.addWaypoint(100, 100);
			naviBot.followPath();
			while(naviBot.isMoving())
			{
				distance.fetchSample(sampledist, 0);
				if(sampledist[0]<0.3)
				{
					Bronchio.rotate(90);
					Bronchio.travel(10);
				}
			}
			*/
			
			/*
				distancefront.fetchSample(sampledistfront, 0);
				Bronchio.travel(100);
				if(sampledistfront[0]<0.20)
				{
					Bronchio.rotate(-90);  //Left Minus, Right Plus
				
					distanceside.fetchSample(sampledistside, 0);
					
					if(sampledistside[0]<=0.3&&sampledistside[0]>=0.1)
					{
						Bronchio.forward();
					}
					else if(sampledistside[0]>0.3)
					{
						Bronchio.rotate(10);
					}
					else if(sampledistside[0]<0.1)
					{
						Bronchio.rotate(-10);
					}
				}
				
				*/
					/*
					else if(sampledistside[0]>0.3)
					{
						Bronchio.rotate(90);
					}
					*/
					
					/*
					if(param == 0)
					{
						param = sampledistside[0];
					}
					
					sb.append(param);
					LCD.drawString(sb.toString(), 1, 1);
					Delay.msDelay(100);
					if(sampledistside[0]<=param+0.03&&sampledistside[0]>=param-0.03)
					{
						Bronchio.forward();
					}
					else if(sampledistside[0]>param+0.05)
					{
						Bronchio.rotate(10);
					}
					else if(sampledistside[0]<param-0.05)
					{
						Bronchio.rotate(-10);
					}
					else if(sampledistside[0]>0.3)
					{
						Bronchio.rotate(90);
					}
					*/
				
			
			
			
			
			
			
			
			/*
				distance.fetchSample(sampledist, 0);
				angle.fetchSample(sampledeg, 0);
				sb.append(sampledeg[0]);
				LCD.drawString(sb.toString(), 1, 1);
				if(sampledeg[0]>270&&sampledeg[0]<300) {
					leftm.forward();
					rightm.forward();
				}
				else
				{
					leftm.forward();
					rightm.backward();
				}
				
				if(sampledist[0]<0.3)
				{
					leftm.forward();
					rightm.backward();
					Delay.msDelay(700); // ~ 90 degrees at this speed
					leftm.forward();
					rightm.forward();
					Delay.msDelay(1000);
				}
				sb.delete(0, 16);
			*/	
		
			/*if(sample[0]>0.3) {
			leftm.forward();
			rightm.forward();
			//distance.fetchSample(sample, 0);
			}else if(sample[0]<0.3)
			{
				leftm.rotate(1440, true);
				rightm.rotate(-1440, true);
			}*/
			
			
			
			/*
			distance.fetchSample(sample, 0);
			sb.append(sample[0]);
			LCD.drawString(sb.toString(), 1, 1);
			Delay.msDelay(100);
			LCD.refresh();
			if(sample[0]>0.5)
			{
				leftm.forward();
			} else{
				leftm.backward();
			}
			sb.delete(0, 16);
			*/
			
		}
	
	}

}
