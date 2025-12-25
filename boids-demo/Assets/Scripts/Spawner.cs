using Boids.Configurations;
using UnityEngine;

namespace Boids
{
    public static class Spawner
    {
        public static Transform Spawn(SpawnerConfiguration spawnerConfig, WorldConfiguration worldConfig)
        {
            var randomPosition = worldConfig.GridCenter + Random.insideUnitSphere * Random.Range(0, worldConfig.GridRadius);
            var transform = Object.Instantiate(spawnerConfig.Prefab, randomPosition, Random.rotation).transform;
            return transform;
        }
    }
}