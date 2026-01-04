using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Jobs
{
    [BurstCompile]
    public struct SpatialMapJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public float CellSize;
        
        public NativeParallelMultiHashMap<int, int>.ParallelWriter SpatialMap;
        
        public void Execute(int index)
        {
            int3 gridPosition = (int3)math.floor(Positions[index] / CellSize);
            int hash = Hash.GetHash(gridPosition);
            SpatialMap.Add(hash, index);
        }
    }
}