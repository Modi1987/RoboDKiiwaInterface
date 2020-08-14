package lbrExampleApplications;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.logging.Logger;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

 
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

/*
 * RoboDK interface, real-time control of KUKA iiwa robot
 * Copyright: Mohammad SAFEEA, 04/August/2020
 * License: MIT license
 * 
 * This is a Sunrise.Workbench program that receives motion
 * commands from RoboDK to move the robot.
 * This program works together with the 'RoboDKiiwaInterface.py'
 * 
 * Below you can find quick start instructions, for more info, please refer to the YouTube
 * video tutorials, a link for which is available in the repository main page.
 * 
 * Hardware setup:
 * 1- KUKA iiwa 7R800 or 14R820
 * 2- External PC (shall be optimised for better performance 
 *                  and and better network communication,
 *                  preferably new, no antivirus no firewall)
 * 3- A network between the PC and the robot, the connector X66
 *    of the robot is used in this program
 * 
 * Required software:
 * 1- RoboDK, with python3.
 * 2- Sunrise.Workbench, this program will be used to synchronise the java
 *    project to the robot.
 * 3- The Sunrise code was tested on (KUKA Sunrise.OS 1.11.0.7)
 * 
 * Setup instructions:
 * 1- Add this class to a new project  using kuka's Sunrsie.Workbench.
 * 2- Add reference to the Direct/SmartServo from inside the (StationSetup.cat) file. 
 * 
 * 
 * Utilisation instructions:
 * 1- Synchronise the project to the controller of the robot.
 * 2- Run this application from the smartPad of the robot,
 *    This application has a time out of 60 seconds, if a connection
 *    is not established during the time interval the program will
 *    turn off, you have to re-start it before initiating a connection again.
 * 3- Start the RoboDK with the python3 script attached.
 * 4- To change the timeout value, change the assignment value for "
 *    the variable "_timeOut".
 * 
 */

public class RoboDKiiwaInterface extends RoboticsAPIApplication
{
    private LBR _lbr;
  
    
    public static double jpos[];

    public static boolean loopFlag;

    public static double jDispMax[];
    public static double updateCycleJointPos[];
    private UDPServer udpServer;
    private static int _port=30003;
    private static int _timeOut=60; 
    private static int _timeOutMillis=_timeOut*1000;
   /////////////////////////////////////
   // Tool attached to the flange
   public static Tool _toolAttachedToLBR;
   /////////////////////////////////////
    ServerSocket ss;
    Socket soc;  

    
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        jpos=new double[7];
        jDispMax=new double[7];
        updateCycleJointPos=new double[7];
        for(int i=0;i<7;i++)
        {
        	jpos[i]=0;
        	jDispMax[i]=0;
        	updateCycleJointPos[i]=0;
        }
        loopFlag=true;
    }
    
    private void attachTheToolToFlange(String TOOL_FRAME,double[] inertialData) {
		// Attach a tool to the robot
		double[] TRANSFORM_OF_TOOL = {0,0,0,0,0,0};
		double MASS= inertialData[0];
		double[] CENTER_OF_MASS_IN_MILLIMETER = { inertialData[1], inertialData[2], inertialData[3] }; 
		LoadData _loadData = new LoadData();
		_loadData.setMass(MASS);
		_loadData.setCenterOfMass(
		CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
		CENTER_OF_MASS_IN_MILLIMETER[2]);
		
		_toolAttachedToLBR = new Tool(TOOL_FRAME, _loadData);
		XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
		TRANSFORM_OF_TOOL[0], TRANSFORM_OF_TOOL[1],
		TRANSFORM_OF_TOOL[2], TRANSFORM_OF_TOOL[3],
		TRANSFORM_OF_TOOL[4], TRANSFORM_OF_TOOL[5]);
		
		ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
		+ "(TCP)", trans);
		_toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
		// Attach tool to the robot
		_toolAttachedToLBR.attachTo(_lbr.getFlange());
		System.out.println("Inertial data for the tool:");
		
		System.out.println("Mass: " + Double.toString(MASS));
		System.out.println("COM x: " + Double.toString(inertialData[1]));
		System.out.println("COM y: " + Double.toString(inertialData[2]));
		System.out.println("COM z: " + Double.toString(inertialData[3]));
	}


    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
        _lbr.move(
        		ptp(Math.PI*10/180, 0.,0.,-Math.PI/2,0.,Math.PI/2, 0.).setJointVelocityRel(0.15));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
        	
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    public void moveToSomePosition()
    {
        _lbr.move(
                ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.));
    }

    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
    	boolean confirmFlag=true;
        // moveToInitialPosition();
    	String info_message="===================";
    	System.out.println(info_message);
    	info_message="An interface for controlling KUKA iiwa robots from RoboDK";
    	System.out.println(info_message);
    	info_message="Borught to you by: Mohamamd Safeea";
    	System.out.println(info_message);
    	info_message="For more info check: [https://github.com/Modi1987]";
    	System.out.println(info_message);
        info_message="You have ";
        info_message=info_message+Integer.toString(_timeOut);
        info_message=info_message+" seconds to establish a connection with iiwa";
        System.out.println(info_message);
        // Sleep for a second
        try {
			Thread.sleep(1000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
        // tool data variables
        String toolsName="RoboDK_tool";
        double[] inertialData_initialConfiguration= new double [11];
        for(int i=0;i<11;i++)
        {
        	inertialData_initialConfiguration[i]=0.0;
        }
        // TCP/IP for adding tool inertia, verifying initial configuration
		try {
			ss= new ServerSocket(_port);
			try
			{
				ss.setSoTimeout(_timeOutMillis);
				soc= ss.accept();
				System.out.println("Connection established successfully");
			}
			catch (Exception e) {
				// TODO: handle exception
				ss.close();
				System.out.println("Time out!!!");
				System.out.println("60 seconds has passed by no connection was received");
				return;
			}
			Scanner scan= new Scanner(soc.getInputStream());
			// Get the inertial data of the tool and the initial configuration
			// from the simulation
			if(scan.hasNextLine())
			{
				String stemp=scan.nextLine();
				boolean flag=parseInitialParameters(stemp,inertialData_initialConfiguration);
				if(flag==false)
				{
					System.out.println("Error in parsing inertial data / initial configuration message");
					soc.close();
					ss.close();
					return;
				}				
			}
			// If data are parsed correctly
			// Verify the initial configuration of the robot
			JointPosition current_J_Pos = new JointPosition(
	                _lbr.getCurrentJointPosition());
			boolean errorFlag=false;
			for(int i=0;i<7;i++)
			{
				double angleDegree=current_J_Pos.get(i)*180/Math.PI;
				double error=angleDegree-
						inertialData_initialConfiguration[i+4];
				if(error*error>1) // if angle difference is more than 1 degree
				{
					errorFlag=true;
				}
			}
			if(errorFlag)
			{
				double [] jAngles=inertialData_initialConfiguration;
				String jointAnglesStr="{"+Double.toString(jAngles[4]);
				for(int i =5;i<11;i++)
				{
					jointAnglesStr=jointAnglesStr+
							"," + Double.toString(jAngles[i]);
				}
				jointAnglesStr=jointAnglesStr+"}";
				String informationText="Current joint angles of the robot does not " +
						"correspond with angles in the simulation, " +
						"do you want the robot to move into simulation configuration," +
						" if you click OK the robot will perform point to point motion in joint space" +
						" to the joints positions (degrees): "+
						jointAnglesStr+
						"! if you choose 'OK' make sure nothing is near the robot " +
						"as not to cause a collision while moving, otherwise hit 'CANCEL'";
		        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
		        if(isCancel==1)
		        {
		        	errorFlag=true;
		        }
		        else
		        {
		        	// move the robot ptp to simulation configuration
		        	String printOut="Mmoving the robot in joint space " +
		        			"into the configuration "+
		        			jointAnglesStr + " [degrees]";
		        	System.out.println(printOut);
		        	JointPosition jAnglesRad=new JointPosition(7);
		        	for(int i=0;i<7;i++)
		        	{
		        		double tempJ_Angle_Rad=jAngles[i+4]*Math.PI/180.0;
		        		jAnglesRad.set(i,tempJ_Angle_Rad);
		        	}
		        	_lbr.move(ptp(jAnglesRad).setJointVelocityRel(0.1));
		        	errorFlag=false;			        
				}
			}
			Thread.sleep(100);
			String ackMessage="";
			if(errorFlag==false)
			{
				ackMessage="ack\n";
			}
			else
			{
				ackMessage="error\n";
			}
			soc.getOutputStream().write(ackMessage.getBytes("US-ASCII"));
			Thread.sleep(100);
			// Terminate connection
			soc.close();
			ss.close();
			if(errorFlag==true)
			{
				System.out.println("Program terminated for you choose 'Cancel'");
				return;
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Error creating server socket");
			System.out.println(e.toString());
			return;
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		if(confirmFlag==true)
		{
			attachTheToolToFlange(toolsName,inertialData_initialConfiguration);
	        int udpPort=30002;
	        udpServer=new UDPServer(udpPort);
	        startDirectServo();
		}
		else
		{
			return;
		}
    }
    
    private static boolean parseInitialParameters(String thestring, double[] parsed_data)
    {
    	StringTokenizer st= new StringTokenizer(thestring,"_");
		int index=0;
		while(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			try {
				parsed_data[index]=Double.parseDouble(temp);
				index=index+1;
			}
			catch(Exception e)
			{
				return false;
			}
		}
		// Check the correct size of the message:
		// 4 numbers for inertial data and 7 numbers for initial configuration
		// total of eleven
		if(index==11)
		{
			return true;
		}
		else
		{
			return false;
		}
    }
    
    private void startDirectServo()
    {
    	boolean doDebugPrints = false;      
        
        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        
        
        for(int i=0;i<7;i++)
        {
        	jpos[i]=initialPosition.get(i);
        }
        DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        _lbr.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();
        
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        double disp;
        double temp;
        double absDisp;
        
        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec

			while(loopFlag==true)
			{

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                //theDirectServoRuntime.updateWithRealtimeSystem();

                if (doDebugPrints)
                {
                	getLogger().info("Current fifth joint position " + jpos[5]);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }


                Thread.sleep(1);
                
                JointPosition currentPos = new JointPosition(
                        _lbr.getCurrentJointPosition());
                
                
                
                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                		
                		jpos[k]=udpServer.jpos[k];
                		double dj=jpos[k]-currentPos.get(k);
                		disp= getTheDisplacment( dj);
                		temp=currentPos.get(k)+disp;
                		absDisp=Math.abs(disp);
                		if(absDisp>jDispMax[k])
                		{
                			jDispMax[k]=absDisp;
                		}
                        destination.set(k, temp);
                        updateCycleJointPos[k]=temp;
                        
                }
                	
                /*terminateFlag=checkCollisionWithEEF(updateCycleJointPos);
                if(terminateFlag==true)
                {
                	dabak.terminateBool=true;
                	dabak.soc.close();
                	dabak.ss.close();
                	
                	break;
                }*/
                
	            theDirectServoRuntime.setDestination(destination);
	                
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

            getLogger().info("Stop the DirectServo motion");

            
        }
        theDirectServoRuntime.stopMotion();

    }
    
    private static double motionMask=1.0;
    double getTheDisplacment(double dj)
    {
    	// limit maximum velocity
    	if(Math.abs(dj)>3.5*Math.PI/180)
    	{
    		dj=0;
    		if(motionMask==1.0)
    		{
    			System.out.println("ERROR: Joint coordinate reference violation");
    			System.out.println("motion will be locked");
    			System.out.println("You shall Terminate program");
    		}
    		motionMask=0.0;
    	}
    	return motionMask*dj;
    	/*
    	double   a=0.07; 
    	double b=a*0.75; 
		double exponenet=-Math.pow(dj/b, 2);
		return Math.signum(dj)*a*(1-Math.exp(exponenet));
		*/
    }


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        RoboDKiiwaInterface app = new RoboDKiiwaInterface();
        app.runApplication();
    }
 
    public class UDPServer implements Runnable{
    	
    	public double[] jpos=new double[7];
    	
    	double stime=0;
		double endtime=0;
		
    	int _port;
    	int vectorSize=7;
    	int floatSize=4;
    	int packetCounter=0;
    	
    	boolean debugMessageSizeError=false;
    	
    	byte[] buffer=new byte[floatSize*vectorSize];
    	UDPServer(int port)
    	{
    		JointPosition currentJointsPos = new JointPosition(
                    _lbr.getCurrentJointPosition());
    		for(int i=0;i<7;i++)
    		{
    			jpos[i]=currentJointsPos.get(i);
    		}
    		Thread t=new Thread(this);
    		t.setDaemon(true);
    		t.start();
    		_port=port;
    	}
    	
    	public void run()
    	{
    		System.out.println("Make sure of a good quality network");
    		System.out.println("Program will terminate if comuncation is distrubted for more than 1 seconds");
			DatagramSocket soc=null;
			try {
				soc = new DatagramSocket(_port);
				soc.setSoTimeout(1000); // 1 seconds, if a time out occurred, turn off program
				DatagramPacket response=new DatagramPacket(buffer, buffer.length);
				while(true)
				{
					soc.receive(response);
					packetCounter=packetCounter+1;
					// String s= new String(buffer,0,response.getLength())
					 // System.out.println(response.getLength());
					if(response.getLength()==floatSize*vectorSize)
					{
						if(packetCounter==1)
						{
							stime=System.currentTimeMillis();
						}
						else
						{
							endtime=System.currentTimeMillis();
						}
						
						byte[] daB=new byte[floatSize];
						int counter=0;
						int jointNum=0;
						while(counter <floatSize*vectorSize)
						{
    						for(int i=0;i<floatSize;i++)
    						{
    							daB[floatSize-1-i]=buffer[counter];
    							counter=counter+1;
    						}
    						jpos[jointNum]=bytesToDouble(daB);
    						jointNum=jointNum+1;	    						
    						//System.out.println(bytesToDouble(daB));
						}
					}
					else
					{
						if(debugMessageSizeError==false)
						{
							String message="Size error, " + Integer.toString(response.getLength());
							System.out.println(message);
							debugMessageSizeError=true;
						}
					}
				}
			} catch (SocketException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				//System.out.println(e.toString());
			} catch (IOException e) {
				// TODO Auto-generated catch block
				// e.printStackTrace();
				System.out.println(e.toString());
				String strTemp= "If you hit the Stop} in RoboDK";
				strTemp=strTemp+" interface disregard the exception, ";
				strTemp=strTemp+"for the program is merely terminating :)";
				System.out.println(strTemp);
			}
			if(soc==null)
			{}
			else
			{
				if(soc.isClosed())
				{}
				else
				{
					soc.close();
				}
			}
			RoboDKiiwaInterface.loopFlag=false; //Turn off main loop 	
			double time=(endtime-stime)/1000;
			double rate=packetCounter/time;
			System.out.println("update rate, packets/second");
			System.out.println(rate);
    	}
    }
    	
    	
    	public double bytesToDouble(byte[] b)
    	{
    		double temp=(double)ByteBuffer.wrap(b).getFloat();
    		return temp;
    	}

}
