using UnityEngine;
using UnityEngine.Serialization;

namespace Boids.Configurations
{
    [CreateAssetMenu(fileName = "SteeringConfig", menuName = "Configuration/Steering")]
    public class SteeringConfiguration : ScriptableObject
    {
        public float SpringWeight = 1.0f;
        public float AlignmentWeight = 0.5f;
        public float SeparationWeight = 1.0f;
        public float CohesionWeight = 0.5f;
    }
}
