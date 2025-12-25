using UnityEngine;

namespace Boids.Configurations
{
    [CreateAssetMenu(fileName = "MovementConfig", menuName = "Configuration/Movement")]
    public class MovementConfiguration : ScriptableObject
    {
        public float MinSpeed = 3.0f;
        public float MaxSpeed = 6.0f;
        public float RotationSpeed = 6.0f;
    }
}
