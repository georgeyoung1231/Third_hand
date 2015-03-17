package kinematics;

import java.lang.Math;
import processing.core.PApplet;
import processing.serial.Serial;

public class InverseKinematics {
	
	double pi = Math.PI;
    int bicep = 48; // bicep length in mm
	int forearm = 75; // forearm length in mm
	int base = 60; // base height in mm
	float bic_sq = bicep*bicep;
	float for_sq = forearm * forearm;
	
	
	double x, y, z, yaw, pitch, row;
	double spaceDistance, yzDistance, cosSpaceDistance, cosSpaceDistanceRadians, cosValue;
	double rotationbyX, rotationKeepXZ, rotationKeepXY;
	double q1, q2, abi, afo;
	double degree0,degree1,degree2,degree3,degree4;

	public double[] ik(double[] pos)
	{
		x = pos[0]; // in mm
		y = pos[1]; // in mm
		z = pos[2];
		yaw = pos[3];
		pitch = pos[4];
		row = pos[5];
		double[] degrees;
		degrees = new double[5];
		spaceDistance = Math.sqrt ((x * x) + (y * y) + (z * z) );
		
		boolean check; 
		check = checkPosition(spaceDistance, x, y, z);
		if (check == false)
		{	degrees[0]=-1;
			return degrees;
		}
		

		
		if (z >= 0)
		{
			yzDistance = Math.sqrt( (y * y) + (z * z) );

			if (y ==0)
			{rotationbyX = 0;}
			else 
			{rotationbyX = - Math.acos (z / yzDistance);}	
			
			q1 = Math.toDegrees(Math.atan( yzDistance / x ));
			cosSpaceDistance= (bic_sq - for_sq + (spaceDistance * spaceDistance))/(2 * bicep * spaceDistance);
			q2 = Math.toDegrees(Math.acos(cosSpaceDistance));
			abi = q1 + q2;			
			
			afo = Math.acos ( (bic_sq + for_sq - (spaceDistance * spaceDistance)) / (2 * bicep * forearm) );
			afo = 180- Math.toDegrees(afo);
			rotationKeepXY = Math.toDegrees(rotationbyX);
			rotationKeepXZ = 85 - (abi - afo) ; //TODO
			
			degree0 = (90 + Math.toDegrees(rotationbyX));
			degree1 = (abi);
			degree2 = (afo);
			degree3 = (rotationKeepXZ);
			degree4 = (-rotationKeepXY);
		}
		
		
		if ( z < 0)
		{
			yzDistance = Math.sqrt( (y * y) + (z * z) );			
			
			if (y ==0)
			{rotationbyX = 0;}
			else 
			{rotationbyX = Math.acos ((-z) / yzDistance);}
//			System.out.println("rotationbyX: " + Math.toDegrees(rotationbyX));
			
			q1 = Math.toDegrees(Math.atan( yzDistance / x ));
			cosSpaceDistance= (bic_sq - for_sq + (spaceDistance * spaceDistance))/(2 * bicep * spaceDistance);
			q2 = Math.toDegrees(Math.acos(cosSpaceDistance));
			System.out.println("spaceDistance: " + spaceDistance);
			System.out.println("yzDistance: " + yzDistance);
			System.out.println("q1: "+ q1);
			System.out.println("cosSpaceDistance: "+ cosSpaceDistance);
			System.out.println("q2: "+ q2);
			abi =  q2 - q1;			
			
			afo = Math.acos ( (bic_sq + for_sq - (spaceDistance * spaceDistance)) / (2 * bicep * forearm) );
			afo = 180- Math.toDegrees(afo);
			rotationKeepXY = -2;
			rotationKeepXZ = 85 - (abi - afo) ;
			
			degree0 = (90 + Math.toDegrees(rotationbyX));
			degree1 = (abi);
			degree2 = (afo);
			degree3 = (rotationKeepXZ);
			degree4 = (-rotationKeepXY);
		}
		
		
		degrees[0] = degree0;
		degrees[1] = degree1;
		degrees[2] = degree2;
		degrees[3] = degree3;
		degrees[4] = degree4;
		return degrees;
	}
	
	public double[] getRelative(double[] destinationPosRot, double[] currentWrist)
	{
		double wristPosition[];
		wristPosition = new double[6];
		wristPosition[0] = destinationPosRot[0] - currentWrist[0];
		wristPosition[1] = destinationPosRot[1] - currentWrist[1];
		wristPosition[2] = destinationPosRot[2] - currentWrist[2];
		wristPosition[3] = 0;
		wristPosition[4] = 0;
		return wristPosition;
	}
	
	private boolean checkPosition(double spaceDistance, double x, double y, double z)
	{
		if (spaceDistance < 27 ||spaceDistance > 123)
		{
			if (spaceDistance < 27)
			{System.out.println("Space too short.");}
			else
			{System.out.println("Space too long.");}
			return false;
		}
		// test for x position
		if ( x < 0)
		{
			System.out.println("X negative.");
			return false;
		}
		if (y < -5 )
		{
			System.out.println("Y negative.");
			return false;
		}
		if (z < -27)
		{
			System.out.println("Z smaller than -27");
			return false;
		}
		return true;
	}
}

