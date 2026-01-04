using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Jobs
{
    // TODO: When should I use the Burst Compiler?
    [BurstCompile]
    public struct BoidSteerJob : IJobParallelFor
    {
        private const float MARGIN = 0.001F;

        // TODO: I should further research what the NativeParallelMultiHashMap is and why we use it.
        [ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialMap;
        [ReadOnly] public float3 GridCenter;
        [ReadOnly] public float GridRadius;
        [ReadOnly] public float CellSize;
        
        [ReadOnly] public float VisionRadius;
        [ReadOnly] public float VisionThreshold;

        [ReadOnly] public float SpringWeight;
        [ReadOnly] public float AlignmentWeight;
        [ReadOnly] public float CohesionWeight;
        [ReadOnly] public float SeparationWeight;

        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        public NativeArray<float3> Steerings;
        
        public void Execute(int index)
        {
            float3 position = Positions[index];
            float3 velocity = Velocities[index];
            float3 velocityNormalized = math.normalize(velocity);
            
            float3 spring = float3.zero;
            float3 alignment = float3.zero;
            float3 separation = float3.zero;
            float3 cohesion = float3.zero;
            int neighbourCount = 0;

            int3 gridPosition = (int3)math.floor(position / CellSize);

            for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++)
            {
                int hash = Hash.GetHash(gridPosition + new int3(x, y, z));
            
                // TODO: I haven't properly digested this code.
                if (!SpatialMap.TryGetFirstValue(hash, out int nIndex, out NativeParallelMultiHashMapIterator<int> iterator)) continue;
                do
                {
                    if (nIndex == index) continue;
            
                    float3 neighbourPosition = Positions[nIndex];
                    float distSq = math.distancesq(position, neighbourPosition);
                    if (distSq > VisionRadius * VisionRadius || distSq < MARGIN) continue;
                    
                    float dot = math.dot(velocityNormalized, math.normalize(neighbourPosition - position));
                    if (dot < VisionThreshold) continue;
                    
                    // Alignment
                    alignment += Velocities[nIndex];
                    
                    // Separation
                    float3 pushForce = position - neighbourPosition;
                    float pushRepulsion = 1 / math.lengthsq(pushForce);
                    separation += math.normalize(pushForce) * pushRepulsion;
                    
                    // Cohesion
                    float3 pullForce = neighbourPosition - position;
                    cohesion += math.normalize(pullForce);
                    
                    neighbourCount++;
                    
                } while (SpatialMap.TryGetNextValue(out nIndex, ref iterator));

                if (math.distancesq(GridCenter, position) > MARGIN)
                {
                    // Spring Force (F = -kx)
                    float springConstant = math.length(velocity) / GridRadius;
                    float3 springDistance = GridCenter - velocity;
                    spring = springDistance * springConstant * SpringWeight;
                }

                if (neighbourCount > 0)
                {
                    alignment = math.normalize(alignment) * AlignmentWeight;
                    separation = math.normalize(separation) * SeparationWeight;
                    cohesion = math.normalize(cohesion) * CohesionWeight;
                }

            }
            
            Steerings[index] = spring + alignment + separation + cohesion;
        }
    }
}