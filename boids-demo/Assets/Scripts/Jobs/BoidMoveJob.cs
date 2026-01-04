using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

namespace Boids.Jobs
{
    [BurstCompile]
    public struct BoidMoveJob : IJobParallelForTransform
    {
        [ReadOnly] public float DeltaTime;
        [ReadOnly] public float MinSpeed;
        [ReadOnly] public float MaxSpeed;
        [ReadOnly] public float RotationSpeed;

        [ReadOnly] public NativeArray<float3> Steerings;
        public NativeArray<float3> Positions;
        public NativeArray<float3> Velocities;
        
        public void Execute(int index, TransformAccess transform)
        {
            float3 velocity = Velocities[index];
            float3 position = Positions[index];
            
            velocity += Steerings[index] * DeltaTime;
            float distance = math.lengthsq(velocity);
            if (distance > MaxSpeed * MaxSpeed)
                velocity = math.normalize(velocity) * MaxSpeed;
            else if (distance < MinSpeed * MinSpeed)
                velocity = math.normalize(velocity) * MinSpeed;
            
            position += velocity * DeltaTime;
            transform.position = position;
            
            Quaternion targetRotation = Quaternion.LookRotation(velocity);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, DeltaTime * RotationSpeed);

            Velocities[index] = velocity;
            Positions[index] = position;
        }
    }
}