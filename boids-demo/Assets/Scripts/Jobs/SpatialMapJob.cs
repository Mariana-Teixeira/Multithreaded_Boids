using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Jobs
{
    // TODO: Research Internal vs Public access points.
    [BurstCompile]
    public struct SpatialMapJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public float CellSize;
        
        // TODO: I should further research what the ParallelWriter is and why we use it.
        public NativeParallelMultiHashMap<int, int>.ParallelWriter SpatialMap;
        
        public void Execute(int index)
        {
            float3 position = Positions[index];
            int3 gridPosition = (int3)math.floor(position / CellSize);
            int hash = Hash.GetHash(gridPosition);
            SpatialMap.Add(hash, index);
        }
    }
}