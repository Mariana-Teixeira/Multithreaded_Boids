using UnityEngine;

namespace Boids.Configurations
{
    [CreateAssetMenu(fileName = "WorldConfig", menuName = "Configuration/World")]
    public class WorldConfiguration : ScriptableObject
    {
        public Vector3 GridCenter = Vector3.zero;
        public float GridRadius = 100.0f;
        public float CellRadius = 10.0f;
        public float GridDiameter => GridRadius * 2.0f;
    }
}
