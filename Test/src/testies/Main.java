package testies;

import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;



public class Main {

	static RegulatedMotor leftm = new EV3LargeRegulatedMotor(MotorPort.A);
	static RegulatedMotor rightm = new EV3LargeRegulatedMotor(MotorPort.B);
	static Port port2 = LocalEV3.get().getPort("S2");
	static SensorModes sens1Ultra = new EV3UltrasonicSensor(port2);
	
	public static void main(String[] args) {
		/*
		System.out.println("Hello faggots");
		LCD.drawString("Plugin Test", 0, 4);
		Delay.msDelay(5000);
		*/
		SampleProvider distance = sens1Ultra.getMode("Distance");
		float[] sample = new float[distance.sampleSize()];
		StringBuffer sb = new StringBuffer(16);
		
		while(true)
		{
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
			
		}
		
		//leftm.rotate(360);
		//rightm.rotate(720);
		/*
		leftm.forward();
		rightm.forward();
		Delay.msDelay(3000);
		leftm.stop();
		rightm.stop();
		*/
	}

}
