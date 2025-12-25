using Unity.Burst;
using Unity.Mathematics;

namespace Boids
{
    [BurstCompile]
    public struct Hash
    {
        // TODO: I don't understand how this functions.
        public static int GetHash(int3 gridPosition)
        {
            unchecked
            {
                return gridPosition.x * 73856093 ^ gridPosition.y * 19349663 ^ gridPosition.z * 83492791;
            }
        }
    }
}