package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import org.usfirst.frc.team3238.robot.MotionProfileExample.PeriodicRunnable;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import ProfileGenerator.Generator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput
{

    /** The Talon we want to motion profile. */
    CANTalon leftLeader = new CANTalon(3), leftFollower = new CANTalon(4),
            rightLeader = new CANTalon(1), rightFollower = new CANTalon(2);
    MotionProfileExample leftController = new MotionProfileExample(leftLeader),
            rightController = new MotionProfileExample(rightLeader);
    Generator g;
    AHRS navX;
    
    Timer t = new Timer();
    boolean e = false;

    int leftCount = 0, rightCount = 0, counterA = 0, controlStatus = 0,
            controlCalls = 0;
    int codesPerRev = 360;

    double[][] leftProf, rightProf, mainProf, straightProfFirst, straightProfSecond;
    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();
    Joystick mainDrive = new Joystick(1);
    PIDController turnController;
    
    public static final double kP = 0.1,
            kI = 0.0,
            kD = 0.00,
            kF = 0.00,
            kToleranceDegrees = 2.0;
    public static final double fTalon = 0.415, pTalon = 3.0;
    
    double rotateToAngleRate = 0;

    public Robot()
    {
        instanceNavX();
        initializeNavPID();
    }
    public void initializeNavPID() {
        turnController = new PIDController(kP, kI, kD, kF, navX, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-0.51625, 0.51625);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
    }
    class PeriodicRunnable implements java.lang.Runnable
    {
        public void run()
        {
            leftLeader.processMotionProfileBuffer();
            rightLeader.processMotionProfileBuffer();
        }
    }

    Notifier _notifer = new Notifier(new PeriodicRunnable());
    public void instanceNavX() {
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch(Exception e) {
            DriverStation.reportError("Error instantiating navx: " + e.getMessage(), false);
        }
    }
    public void robotInit()
    {
        declareEncodersExist();
        setEncoderInversions();
        setFollowerTalons();
//        setLiftProfilesLocally();
        setSideLiftProfilesLocally();
    }

    public void declareEncodersExist()
    {
        leftLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftLeader.configEncoderCodesPerRev(codesPerRev);
        rightLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightLeader.configEncoderCodesPerRev(codesPerRev);
    }
    public void setSideLiftProfilesLocally() {
        
        straightProfFirst = Profiles.FirstProfile.Points;
        straightProfSecond = Profiles.SecondProfile.Points;
//        DriverStation.reportError(g.getStraightProfile(3.0).toString(), false);
//        straightProfFirst = g.getStraightProfile(ProfileGenerator.ConstantsMP.kFirstForwardMove);
//        straightProfSecond = g.getStraightProfile(ProfileGenerator.ConstantsMP.kSecondForwardMove);
    }
    public void setLiftProfilesLocally()
    {
        g = new Generator();
        g.generateSideProfile();
        leftProf = g.getLeftSideLift();
        rightProf = g.getRightSideLift();
        mainProf = g.getMainLift();
        straightProfFirst = g.getStraightProfile(ProfileGenerator.ConstantsMP.kFirstForwardMove);
        straightProfSecond = g.getStraightProfile(ProfileGenerator.ConstantsMP.kSecondForwardMove);
        
    }
//    boolean e = false;
    public boolean startMotionProfilesLoop()
    {
        pollProfileStatus();
        DriverStation.reportWarning(
                "auto periodic: MP status = " + _status.toString(), false);
        
        switch(controlStatus)
        {
            case 0:
                DriverStation.reportWarning("auto case 0", false);
                _notifer.startPeriodic(0.005);
                leftController.fillFed();
                rightController.fillFed();
                leftLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                rightLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                controlStatus++;
                return e;
            default:
                DriverStation.reportWarning("auto case 1", false);
                // _talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                if(!e)
                {

                    double motorOutput = leftLeader.getOutputVoltage()
                            / leftLeader.getBusVoltage();
                    double altMotorOutput = rightLeader.getOutputVoltage()
                            / rightLeader.getBusVoltage();
                    System.out.println("motor output: " + motorOutput);
                    System.out.println("motor speed: " + leftLeader.getSpeed());
                    System.out.println("error in native units: "
                            + leftLeader.getClosedLoopError());
                }

                
                if(_status.activePointValid && _status.activePoint.isLastPoint)
                {
                    SmartDashboard.putNumber("Loops", controlCalls);
                    DriverStation.reportWarning(
                            "auto case default: profile done", false);
                    leftLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    rightLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    DriverStation.reportWarning("# auto loops: " + controlCalls,
                            false);
                    System.out.println(leftLeader.getPosition());
                    System.out.println(rightLeader.getPosition());
                    e = true;
                }

                controlCalls++;

                return e;
        }
    }
    public void setSpeedForward() {
        leftLeader.set(-0.5);
        rightLeader.set(0.5);
    }
    public void prepEncodersForProfile()
    {
        resetEncoderPosition();
        enableMotionProfileMode();
        setEncoderInversions();
        setEncoderPID();
        resetDiagnostics();
    }
    public void disableMotors()
    {
        leftLeader.changeControlMode(TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(TalonControlMode.PercentVbus);
        leftLeader.set(0);
        rightLeader.set(0);
        leftController.reset();
        rightController.reset();
    }

    public void setFollowerTalons()
    {
        leftFollower.changeControlMode(TalonControlMode.Follower);
        leftFollower.set(leftLeader.getDeviceID());
        rightFollower.changeControlMode(TalonControlMode.Follower);
        rightFollower.set(rightLeader.getDeviceID());
    }

    public void setEncoderPID()
    {
        leftLeader.setF(fTalon);
        leftLeader.setP(pTalon);
        leftLeader.setI(0.0);
        rightLeader.setP(pTalon);
        leftLeader.setD(0.0);
        rightLeader.setF(fTalon);
        rightLeader.setI(0.0);
        rightLeader.setD(0.0);
    }

    public void resetEncoderPosition()
    {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }

    public void setEncoderInversions()
    {
        rightLeader.reverseOutput(false);
        leftLeader.reverseSensor(false);
        rightLeader.reverseSensor(true);
        leftLeader.reverseOutput(true);
    }

    public void enableMotionProfileMode()
    {
        leftLeader.clearMotionProfileTrajectories();
        leftLeader.changeControlMode(TalonControlMode.MotionProfile);
        leftLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        leftLeader.changeMotionControlFramePeriod(5);
        rightLeader.clearMotionProfileTrajectories();
        rightLeader.changeControlMode(TalonControlMode.MotionProfile);
        rightLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        rightLeader.changeMotionControlFramePeriod(5);

    }

    public void pollProfileStatus()
    {
        leftLeader.getMotionProfileStatus(_status);
        rightLeader.getMotionProfileStatus(_status);
    }

    public void resetDiagnostics()
    {
        controlStatus = 0;
        controlCalls = 0;
        e = false;
    }

    public void setSideProfile()
    {
        leftController.setFedProfile(leftProf);
        rightController.setFedProfile(rightProf);
    }

    public void setStraightProfile()
    {
        leftController.setFedProfile(mainProf);
        rightController.setFedProfile(mainProf);
    }
    public void setLayeredProfile(int n) {
        if(n == 1) {
          leftController.setFedProfile(straightProfFirst); 

          rightController.setFedProfile(straightProfFirst); 
        } else if(n == 2) {
            leftController.setFedProfile(straightProfSecond); 

            rightController.setFedProfile(straightProfSecond); 
        }
    }
    public void setMotorsDriveMode()
    {
        leftLeader.changeControlMode(TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(TalonControlMode.PercentVbus);
    }

    public void resetTimer()
    {
        t.reset();
        t.start();
    }

    public void joystickControl()
    {
        double twist = 0, y = 0;
        if(Math.abs(mainDrive.getTwist()) > 0.15)
        {
            twist = mainDrive.getTwist();

        }
        if(Math.abs(mainDrive.getY()) > 0.15)
        {
            y = mainDrive.getY();
        }
        leftLeader.set(-y + twist);
        rightLeader.set(y + twist);
    }

    public void encoderDiagnostics() {
        if(t.get() > 0.1)
        {
            // System.out.println(t.get());
            leftCount += leftLeader.getPosition();
            rightCount += rightLeader.getPosition();
            counterA++;
            System.out.println(
                    (leftCount / counterA) + " " + (rightCount / counterA));
            t.reset();
            leftLeader.setPosition(0);
            rightLeader.setPosition(0);
        }
    }
    
    public void autonomousInit()
    {
//        prepEncodersForProfile();
//        setSideProfile();
        setMotorsDriveMode();
    }

    public void autonomousPeriodic()
    {
        setSpeedForward();
//        startMotionProfilesLoop();
    }
    
    public void testInit()
    {
        resetDiagnostics();
        prepEncodersForProfile();
        setLayeredProfile(1);
        int n=0;
        valid_counter = 0;
        firstStage = false;
        secondStage = false;
        t.reset();
        iFix = 0.01;
        turnController.setPID(kP, kI, kD, kF);
        adjusted = false;
        //
    }
    double iFix = kP;
    boolean adjusted = false;
    boolean firstStage = false, secondStage = false;;
    public void testPeriodic()
    {
//        startMotionProfilesLoop();
        DriverStation.reportError("Angle: " + navX.getAngle(), false);
        if(!firstStage && startMotionProfilesLoop()) {
            firstStage = true;
            setMotorsDriveMode();
            t.reset();
            t.start();
        }
        if(firstStage && !secondStage) {
//            if(t.get() > 1.6) {
//                turnController.setPID(iFix, kI, kD,kF);
////                iFix+=0.01;
//                iFix+=kP/4;
//                t.reset();
//                t.start();
//                adjusted = true;
//            }
//            if(adjusted && t.get() > 0.05) {
//             
//                iFix+=kP/4;
//                turnController.setPID(iFix, kI, kD, kF);
//                t.reset();
//                t.start();
//            }
            if(turnToDesiredAngle() > 42) {
                setLayeredProfile(2);
                secondStage = true;
                resetDiagnostics();
                prepEncodersForProfile();
                setLayeredProfile(2);
            }
        }
        if(secondStage) {
            startMotionProfilesLoop();
        }
        
    }

    public void teleopInit()
    {
        setMotorsDriveMode();
        resetEncoderPosition();
        resetTimer();
        setLayeredProfile(0);
    }

    public void teleopPeriodic()
    {
        joystickControl();
//        encoderDiagnostics();
//        turnToDesiredAngle();
        DriverStation.reportError("Encoder Pos L : " + leftLeader.getEncPosition() + "\nEPR: " + rightLeader.getEncPosition(), false);
    }
    boolean rotateToAngle = false;
    int valid_counter = 0;
    public int turnToDesiredAngle() {
//        boolean done = false;
        double currentRotationRate = 0;
        if(!turnController.isEnabled()) {
        turnController.setSetpoint(-60.0f);
        turnController.enable();
        }
        if(turnController.onTarget()) valid_counter++;
        currentRotationRate = rotateToAngleRate;
        rightLeader.set(currentRotationRate);
        leftLeader.set(currentRotationRate);
        DriverStation.reportWarning(""+navX.getAngle(),false);
        return valid_counter;
    }

    public void disabledPeriodic()
    {
        disableMotors();
        navX.zeroYaw();
    }

    @Override
    public void pidWrite(double output)
    {
        rotateToAngleRate = output;
        // TODO Auto-generated method stub
        
    }
}