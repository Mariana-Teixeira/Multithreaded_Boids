using UnityEngine;

[CreateAssetMenu(fileName = "BoidData", menuName = "Data/Boid")]
public class BoidData : ScriptableObject
{
    [Header("Move")]
    public float MinSpeed;
    public float MaxSpeed;
    public float FlockAcceleration;
    public float ContainmentAcceleration;
            
    [Header("Probes")]
    [Tooltip("Probe Length is calculated by multiplying the velocity vector by this multiplier.")]
    public float ProbeLengthMultiplier;
    [Tooltip("Probe Angle defines the tilt rotation of the four directional probes.")]
    public float ProbeAngle;

    [Header("Steering")]
    public float SeparationRadius;
    [Range(-1, 1)]
    public float SeparationDot;
    public float SeparationWeight;

    [Space]
    public float CohesionRadius;
    [Range(-1, 1)]
    public float CohesionDot;
    public float CohesionWeight;

    [Space]
    public float AlignmentRadius;
    [Range(-1, 1)]
    public float AlignmentDot;
    public float AlignmentWeight;
}
