package thirdhand;

import processing.core.PApplet;

import oscP5.*;
import netP5.*;
import processing.core.PApplet;
import processing.serial.Serial;
import kinematics.InverseKinematics;
import kinematics.ForwardKinematics;
import kinematics.KinematicsTest;

public class ThirdHand extends PApplet {

	Serial arduino;
	int motor0, motor1, motor2, motor3, motor4;
	int mode;
	double[] degreesMotor;
	double[] initDegreesMotor;
	double[] currentDegreesMotor;
	double[] destinationPosRot;
	double[] currentPosRot;
	double[] relativePosRot;
	double[] vectortoCurrentPos;
	double[] vectortoCurrentWrist;
	double[] currentWrist;
	
	double [] test1, test2, testIK, testFK;
	double unitt =  155/341.46634;
	
	InverseKinematics ik = new InverseKinematics();
	ForwardKinematics fk = new ForwardKinematics();
	KinematicsTest kt= new KinematicsTest();
	
	int count;

	OscP5 oscP5;
	NetAddress myRemoteLocation;
	float mat4[] = new float[16];
	
	public void setup() {
		arduino = new Serial(this, "/dev/tty.usbmodem1411" ,9600);
		arduino.bufferUntil('\n');
		//size(500,500, P3D);
		
		// mode 0 for test IK, give position,
		// mode 1 for test FK, give rotations,
		// mode 2 for two positions, give init rotations, init position, and final position,
		// mode 3 for test OSC, give init rotation.
		mode = 3;
		
		stroke(255, 0, 0);
		oscP5 = new OscP5(this,5000);
		
		myRemoteLocation = new NetAddress("192.168.0.105",5000);
		
		destinationPosRot = new double[6];
		currentPosRot = new double[6];
		vectortoCurrentPos = new double[6];
		degreesMotor = new double[5];
		initDegreesMotor = new double[5];
		currentDegreesMotor = new double[5];
		relativePosRot = new double[6];
		vectortoCurrentWrist = new double[6];
		
		testIK = new double[6];
		testFK = new double[5];
		
		count = 0;
		
		initDegreesMotor[0]= 90;
		initDegreesMotor[1]= 77;
		initDegreesMotor[2]= 133;
		initDegreesMotor[3]= 136;
		initDegreesMotor[4]= 2;
		
		currentDegreesMotor[0]= 90;
		currentDegreesMotor[1]= 77;
		currentDegreesMotor[2]= 133;
		currentDegreesMotor[3]= 136;
		currentDegreesMotor[4]= 2;
		
		// for mode 0
		testIK[0] = 20.757768720122577;
		testIK[1] = 0;
		testIK[2] = -15.40805483193683;
		testIK[3] = 0;
		testIK[4] = 0;
		testIK[5] = 0;
		// for mode 1
		testFK[0] = 25;
		testFK[1] = 79;
		testFK[2] = 89;
		testFK[3] = 95;
		testFK[4] = 64;
		// for mode 2
		currentPosRot[0] = 0;
		currentPosRot[1] = 0;
		currentPosRot[2] = 0;
		currentPosRot[3] = 0;
		currentPosRot[4] = 0;
		currentPosRot[5] = 0;
		destinationPosRot[0] = 30;
		destinationPosRot[1] = 30;
		destinationPosRot[2] = 30;
		destinationPosRot[3] = 90;
		destinationPosRot[4] = 90;
		destinationPosRot[5] = 90;
		
		
	}

	public void draw() {
		if (mode == 0)
		{
			test1 = kt.testIK(testIK);
			testFK[0] = test1[0];
			testFK[1] = test1[1];
			testFK[2] = test1[2];
			testFK[3] = test1[3];
			testFK[4] = test1[4];
			kt.testFK(testFK);
			arduinoWrite(testFK);
		}
		if (mode == 1)
		{
			test2 = kt.testFK(testFK);
			testIK[0] = -test2[0];
			testIK[1] = -test2[1];
			testIK[2] = -test2[2];
			testIK[3] = test2[3];
			testIK[4] = test2[4];
			kt.testIK(testIK);
			arduinoWrite(testFK);
		}
		if(mode ==2)
		{
			if ((currentPosRot[0]!= destinationPosRot[0]) || (currentPosRot[1]!= destinationPosRot[1]) || (currentPosRot[2]!= destinationPosRot[2]) )
			{
				vectortoCurrentWrist = fk.getVectorWrist(currentDegreesMotor);
				currentWrist = fk.getWrist(currentPosRot, vectortoCurrentWrist);
				relativePosRot = ik.getRelative(destinationPosRot, currentWrist);
				
				System.out.println("vectortoCurrentWrist: " + vectortoCurrentWrist[0] + "  "+ vectortoCurrentWrist[1] + "  "+ vectortoCurrentWrist[2]);
				System.out.println("currentWrist: " + currentWrist[0] + "  "+ currentWrist[1] + "  "+ currentWrist[2]);
				System.out.println("relativePosRot: " + relativePosRot[0] + "  "+ relativePosRot[1] + "  "+ relativePosRot[2]);
				
				degreesMotor = ik.ik(relativePosRot);
				vectortoCurrentPos[0] = - vectortoCurrentWrist[0];
				vectortoCurrentPos[1] = - vectortoCurrentWrist[1];
				vectortoCurrentPos[2] = - vectortoCurrentWrist[2];
				vectortoCurrentPos[3] = 0;
				vectortoCurrentPos[4] = 0;
				degreesMotor = ik.ik(relativePosRot);
			
				currentPosRot[0] = currentWrist[0] + relativePosRot[0];
				currentPosRot[1] = currentWrist[1] + relativePosRot[1];
				currentPosRot[2] = currentWrist[2] + relativePosRot[2];
		
				currentDegreesMotor[0] = degreesMotor[0];
				currentDegreesMotor[1] = degreesMotor[1];
				currentDegreesMotor[2] = degreesMotor[2];
				currentDegreesMotor[3] = degreesMotor[3];
				currentDegreesMotor[4] = degreesMotor[4];
				printDegrees(degreesMotor);
				System.out.println("final position: " + currentPosRot[0] + "  "+ currentPosRot[1] + "  "+ currentPosRot[2] + "  ");
//				arduinoWrite(degreesMotor);
			}
		}
		if (mode ==3)
		{}	
	}
	
	void oscEvent(OscMessage theOscMessage) 
	{ 
		if (mode == 3)
		{
			double x, y, z;
			x = 0;
			y = 0;
			z = 0;
		    for(int i=0; i < 16; ++i){
			    mat4[i] = theOscMessage.get(i).floatValue();
		    }
		    // print out the message
		    println("OSC Message Recieved: ");
		    print(theOscMessage.addrPattern() + " ");
		    for(int i=0; i < 16; ++i){	
			    if (i ==12)
			    {x = mat4[i] * unitt;println("x: " +mat4[i] * unitt);}
	//		    if (i ==13)
	//		    {y = mat4[i] * unitt;println("y: " +mat4[i] * unitt);}
	//		    if (i ==14)
	//		    {z = mat4[i] * unitt;println("z: " +mat4[i] * unitt);}	
		    }
		    if (count == 0)
		    {
		    	destinationPosRot[0] = 0;
				destinationPosRot[1] = y;
				destinationPosRot[2] = z;
				destinationPosRot[3] = 90;
				destinationPosRot[4] = 90;
				destinationPosRot[5] = 90;
				currentPosRot[0] = x;
				currentPosRot[1] = y;
				currentPosRot[2] = z;
				currentPosRot[3] = 0;
				currentPosRot[4] = 0;
				currentPosRot[5] = 0;
				count ++;
		    }
		    else
		    {
		    	currentPosRot[0] = x;
				currentPosRot[1] = y;
				currentPosRot[2] = z;
				currentPosRot[3] = 0;
				currentPosRot[4] = 0;
				currentPosRot[5] = 0;
				
		    	if ((currentPosRot[0]!= destinationPosRot[0]) || (currentPosRot[1]!= destinationPosRot[1]) || (currentPosRot[2]!= destinationPosRot[2]) )
				{
		    		
					
					vectortoCurrentWrist = fk.getVectorWrist(currentDegreesMotor);
					currentWrist = fk.getWrist(currentPosRot, vectortoCurrentWrist);
					relativePosRot = ik.getRelative(destinationPosRot, currentWrist);
					
					System.out.println("vectortoCurrentWrist: " + vectortoCurrentWrist[0] + "  "+ vectortoCurrentWrist[1] + "  "+ vectortoCurrentWrist[2]);
					System.out.println("currentWrist: " + currentWrist[0] + "  "+ currentWrist[1] + "  "+ currentWrist[2]);
					System.out.println("relativePosRot: " + relativePosRot[0] + "  "+ relativePosRot[1] + "  "+ relativePosRot[2]);
					if(ik.ik(relativePosRot)[0] != -1){
						System.out.println(ik.ik(relativePosRot)[0]);
						System.out.println("to arduinooooooooooo");
						degreesMotor = ik.ik(relativePosRot);
						vectortoCurrentPos[0] = - vectortoCurrentWrist[0];
						vectortoCurrentPos[1] = - vectortoCurrentWrist[1];
						vectortoCurrentPos[2] = - vectortoCurrentWrist[2];
						vectortoCurrentPos[3] = 0;
						vectortoCurrentPos[4] = 0;
	//					degreesMotor = ik.ik(relativePosRot);
					
						currentPosRot[0] = currentWrist[0] + relativePosRot[0];
						currentPosRot[1] = currentWrist[1] + relativePosRot[1];
						currentPosRot[2] = currentWrist[2] + relativePosRot[2];
				
						currentDegreesMotor[0] = degreesMotor[0];
						currentDegreesMotor[1] = degreesMotor[1];
						currentDegreesMotor[2] = degreesMotor[2];
						currentDegreesMotor[3] = degreesMotor[3];
						currentDegreesMotor[4] = degreesMotor[4];
						
						printDegrees(degreesMotor);
						System.out.println("final position: " + currentPosRot[0] + "  "+ currentPosRot[1] + "  "+ currentPosRot[2] + "  ");
						arduinoWrite(degreesMotor);
					}
				}
		    }
	    }
	    
	}
	
	private void arduinoWrite (double[] degreesMotor)
	{
		double toArduino[];
		toArduino = new double[5];
		toArduino[0] = (int)degreesMotor[0];
		toArduino[1] = (int)degreesMotor[1];
		toArduino[2] = (int)degreesMotor[2];
		toArduino[3] = (int)degreesMotor[3];
		toArduino[4] = (int)degreesMotor[4];
		System.out.println("to arduino: "+ toArduino[0] + "  "+toArduino[1] + "  "+toArduino[2] + "  "+toArduino[3] + "  "+toArduino[4] + "  ");
		arduino.write(0xff);
		arduino.write(0xfe);
		arduino.write((int)toArduino[0]);
		arduino.write((int)toArduino[1]);
		arduino.write((int)toArduino[2]);
		arduino.write((int)toArduino[3]);
		arduino.write((int)toArduino[4]);
	}
	
	private void printDegrees (double[] degreesMotor)
	{
		double print[];
		print = new double[5];
		print[0] = (int)degreesMotor[0];
		print[1] = (int)degreesMotor[1];
		print[2] = (int)degreesMotor[2];
		print[3] = (int)degreesMotor[3];
		print[4] = (int)degreesMotor[4];
		System.out.println("to arduino: "+ print[0] + "  "+print[1] + "  "+print[2] + "  "+print[3] + "  "+print[4] + "  ");
	}
}
