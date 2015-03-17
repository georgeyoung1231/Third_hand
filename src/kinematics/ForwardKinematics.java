package kinematics;

import java.lang.Math;

public class ForwardKinematics {
	double pi = Math.PI;
    int bicep = 48; // bicep length in mm
	int forearm = 75; // forearm length in mm
	int base = 60; // base height in mm
	float bic_sq = bicep*bicep;
	float for_sq = forearm * forearm;
	
	
	double x, y, z, yaw, pitch, row;
	double spaceDistance, yzDistance, cosSpaceDistance, cosSpaceDistanceRadians;
	double rotationbyX, rotationKeepXZ, rotationKeepXY;
	double q1, q2, abi, afo;
	double degree0,degree1,degree2,degree3,degree4;
	double currentdegree0,currentdegree1,currentdegree2,currentdegree3,currentdegree4;
	
	public double[] getVectorWrist(double[] currentDegreesMotor)
	{
		currentdegree0 = currentDegreesMotor[0]; 
		currentdegree1 = currentDegreesMotor[1]; 
		currentdegree2 = currentDegreesMotor[2]; 
		currentdegree3 = currentDegreesMotor[3]; 
		currentdegree4 = currentDegreesMotor[4]; 
		
		double degreeBetween = Math.toRadians(180 - currentdegree2);
		spaceDistance = Math.sqrt(bic_sq + for_sq - 2 * bicep * forearm *Math.cos(degreeBetween));
		cosSpaceDistance= (bic_sq - for_sq + (spaceDistance * spaceDistance))/(2 * bicep * spaceDistance);
		q2 = Math.toDegrees(Math.acos(cosSpaceDistance));
		
		if (currentdegree1 >= q2)
		{q1 = currentdegree1 - q2;}
		else
		{q1 = q2 - currentdegree1;}
		yzDistance = spaceDistance * Math.sin(Math.toRadians(q1));
		
		x = spaceDistance * Math.cos(Math.toRadians(q1));
		double rotationbyX = currentdegree0 ;
		if(currentdegree1 >= q2) 
		{y = yzDistance * Math.cos(Math.toRadians(rotationbyX));}
		else
		{y = -yzDistance * Math.cos(Math.toRadians(rotationbyX));}
		if(currentdegree1 >= q2)
		{z = yzDistance * Math.sin(Math.toRadians(rotationbyX));}
		else
		{z = - yzDistance * Math.sin(Math.toRadians(rotationbyX));}
		
		double[] vectortoCurrentWrist; 
		vectortoCurrentWrist = new double[6];
		vectortoCurrentWrist[0] = -x;
		vectortoCurrentWrist[1] = -y;
		vectortoCurrentWrist[2] = -z;
		vectortoCurrentWrist[3] = 0;
		vectortoCurrentWrist[4] = 0;
		vectortoCurrentWrist[5] = 0;
		return vectortoCurrentWrist;
	}
	
	public double[] getWrist(double[] currentPosRot, double[] vectortoCurrentWrist)
	{
		double wristPosition[];
		wristPosition = new double[6];
		wristPosition[0] = currentPosRot[0] + vectortoCurrentWrist[0];
		wristPosition[1] = currentPosRot[1] + vectortoCurrentWrist[1];
		wristPosition[2] = currentPosRot[2] + vectortoCurrentWrist[2];
		wristPosition[3] = 0;
		wristPosition[4] = 0;
		return wristPosition;
	}

}
