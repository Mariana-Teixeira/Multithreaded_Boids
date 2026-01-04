using UnityEngine;

namespace Boids
{
    public static class Spawner
    {
        public static Transform Spawn(Vector3 gridCenter, float gridRadius, GameObject prefab)
        {
            var randomPosition = gridCenter + Random.insideUnitSphere * Random.Range(0, gridRadius);
            var transform = Object.Instantiate(prefab, randomPosition, Random.rotation).transform;
            return transform;
        }
    }
}