using UnityEngine;

[CreateAssetMenu(fileName = "SteeringConfig", menuName = "Configuration/Steering")]
public class SteeringConfiguration : ScriptableObject
{
    [Range(0, 1)] public float SpringForce = 1.0f;
    [Range(0, 1)] public float AlignmentForce = 0.5f;
    [Range(0, 1)] public float SeparationForce = 1.0f;
    [Range(0, 1)] public float CohesionForce = 0.5f;
}