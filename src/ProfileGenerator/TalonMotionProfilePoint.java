package ProfileGenerator;

public class TalonMotionProfilePoint {

    //constructors
    public TalonMotionProfilePoint(double positionInRevolutions, double speedInRPMs, double timestepInMilliseconds)
    {
        _positionInRevolutions = positionInRevolutions;
        _speedInRPMs = speedInRPMs;
        _timestepInMilliseconds = timestepInMilliseconds;
    }

    public TalonMotionProfilePoint()
    {
        _positionInRevolutions = 0;
        _speedInRPMs = 0;
        _timestepInMilliseconds = 0;
    }
    
    private double _positionInRevolutions;
    public double GetPositionInRevolutions()
    {
        return _positionInRevolutions;
    }
    public void SetPositionInRevolutions(double value)
    {
        _positionInRevolutions = value;
    }
    
    private double _speedInRPMs;
    public double GetSpeedInRPMs()
    {
        return _speedInRPMs;
    }
    public void SetSpeedInRPMs(double value)
    {
        _speedInRPMs = value;
    }

    private double _timestepInMilliseconds;
    public double GetTimestepInMilliseconds()
    {
        return _timestepInMilliseconds;
    }
    public void TimestepInMilliseconds(double value)
    {
        _timestepInMilliseconds = value;
    }
    
}
