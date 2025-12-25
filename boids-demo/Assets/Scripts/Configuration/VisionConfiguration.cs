using UnityEngine;

namespace Boids.Configurations
{
    [CreateAssetMenu(fileName = "VisionConfig", menuName = "Configuration/Vision")]
    public class VisionConfiguration : ScriptableObject
    {
        public float VisionRadius = 20.0f;
        public float VisionAngle = 60.0f; // Half the vision cone angle.
    }
}