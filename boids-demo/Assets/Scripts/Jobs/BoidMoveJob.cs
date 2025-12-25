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

        [ReadOnly] public NativeArray<float3> SteeringForces;
        
        public NativeArray<float3> Positions;
        public NativeArray<float3> Velocities;
        
        public void Execute(int index, TransformAccess transform)
        {
            float3 currentVelocity = Velocities[index];
            
            currentVelocity += SteeringForces[index] * DeltaTime;
            float distance = math.lengthsq(currentVelocity);
            if (distance > MaxSpeed * MaxSpeed)
                currentVelocity = math.normalize(currentVelocity) * MaxSpeed;
            else if (distance < MinSpeed * MinSpeed)
                currentVelocity = math.normalize(currentVelocity) * MinSpeed;
            
            Positions[index] += currentVelocity * DeltaTime;
            transform.position = Positions[index];
            
            Quaternion targetRotation = Quaternion.LookRotation(currentVelocity);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, DeltaTime * RotationSpeed);
            
            Velocities[index] = currentVelocity;
        }
    }
}