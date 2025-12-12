using System;

[Serializable]
public struct BoidConfiguration
{
    public WorldConfiguration World;
    public VisionConfiguration Vision;
    public MovementConfiguration Movement;
    public SteeringConfiguration Steering;
}