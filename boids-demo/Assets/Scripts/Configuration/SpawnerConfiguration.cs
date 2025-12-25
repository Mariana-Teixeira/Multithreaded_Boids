using UnityEngine;

namespace Boids.Configurations
{
    [CreateAssetMenu(fileName = "SpawnerConfig", menuName = "Configuration/Spawner")]
    public class SpawnerConfiguration : ScriptableObject
    {
        public GameObject Prefab;
        public int Count = 500;
    }
}