using System;
using System.Net;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

namespace Demo.Boids
{
    public class DemoManager : MonoBehaviour
    {
        [Serializable]
        private struct WorldData
        {
            public float WorldRadius;
            public float SpawnRadius;
            public int Count;
        }
        
        [Serializable]
        private struct BoidData
        {
            [Header("Move")]
            public float MinSpeed;
            public float MaxSpeed;

            [Header("Spatial Grid (WIP)")]
            public float CellSize;
            
            [Header("Probes")]
            public float ProbeLength;
            public float ProbeAngle;

            [Header("Steering")]
            public float SeparationRadius;
            [Range(-1, 1)]
            public float SeparationDot;
            public float SeparationWeight;

            [Space]
            public float CohesionRadius;
            [Range(-1, 1)]
            public float CohesionDot;
            public float CohesionWeight;

            [Space]
            public float AlignmentRadius;
            [Range(-1, 1)]
            public float AlignmentDot;
            public float AlignmentWeight;
        }
        
        [SerializeField]
        private GameObject m_prefab;

        [Space, SerializeField]
        private WorldData m_worldData;
        [Space, SerializeField]
        private BoidData m_boidData;
        
        private TransformAccessArray m_transforms;
        private NativeArray<float3> m_positions;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_steerings;
        // TODO: Look information about NativeParallelMultiHashMap.
        private NativeParallelMultiHashMap<int, int> m_spatialHashMap;
        private NativeArray<float3> m_probes;

        private const int PROBE_OFFSET = 5;
    
        private void Awake()
        {
            m_positions = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_steerings = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_spatialHashMap = new NativeParallelMultiHashMap<int, int>(m_worldData.Count, Allocator.Persistent);
            m_probes = new NativeArray<float3>(m_worldData.Count * PROBE_OFFSET, Allocator.Persistent);
        }

        private void OnDestroy()
        {
            m_transforms.Dispose();
            m_positions.Dispose();
            m_velocities.Dispose();
            m_steerings.Dispose();
            m_spatialHashMap.Dispose();
            m_probes.Dispose();
        }

        private void Start()
        {
            InstantiateBoids();
        }
        
        private void InstantiateBoids()
        {
            Transform[] transforms = new Transform[m_worldData.Count];
            for (int index = 0; index < m_worldData.Count; index++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * m_worldData.SpawnRadius;
                GameObject boid = Instantiate(m_prefab, randomPosition, Quaternion.identity);
                boid.name = $"Boids_{index}";

                transforms[index] = boid.transform;
                m_positions[index] = boid.transform.position;
            }
            m_transforms = new TransformAccessArray(transforms);
        }

        private void Update()
        {
            m_spatialHashMap.Clear();

            UpdateHashJob hashJob = new UpdateHashJob
            {
                Positions = m_positions,
                SpatialHashMap = m_spatialHashMap.AsParallelWriter(),
                CellSize = m_boidData.CellSize
            };
            JobHandle gridHandle = hashJob.Schedule(m_worldData.Count, 64);
            
            UpdateProbesJob probesJob = new UpdateProbesJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                ProbeLength = m_boidData.ProbeLength,
                ProbeAngle = m_boidData.ProbeAngle
            };
            JobHandle probesHandle = probesJob.Schedule(m_worldData.Count, 64);
            
            JobHandle setupHandle = JobHandle.CombineDependencies(gridHandle, probesHandle);
            
            FlockSteeringJob flockJob = new FlockSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings,
                SpatialHashMap = m_spatialHashMap,
                CellSize = m_boidData.CellSize,
                SeparationRadius = m_boidData.SeparationRadius,
                SeparationThreshold = m_boidData.SeparationDot,
                SeparationWeight = m_boidData.SeparationWeight,
                CohesionRadius = m_boidData.CohesionRadius,
                CohesionThreshold = m_boidData.CohesionDot,
                CohesionWeight = m_boidData.CohesionWeight,
                AlignmentRadius = m_boidData.AlignmentRadius,
                AlignmentThreshold = m_boidData.AlignmentDot,
                AlignmentWeight = m_boidData.AlignmentWeight,
                MaxSpeed = m_boidData.MaxSpeed
            };
            JobHandle flockHandle = flockJob.Schedule(m_worldData.Count, 64, setupHandle);
            
            ContainmentSteeringJob containmentJob = new ContainmentSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings,
                Probes = m_probes,
                WorldRadius = m_worldData.WorldRadius,
                MaxSpeed = m_boidData.MaxSpeed
            };
            JobHandle containmentHandle = containmentJob.Schedule(m_worldData.Count, 64, flockHandle);
            
            UpdateMovementJob movementJob = new UpdateMovementJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings,
                MaxSpeed = m_boidData.MaxSpeed,
                DeltaTime = Time.deltaTime
            };
            JobHandle moveHandle = movementJob.Schedule(m_transforms, containmentHandle);
            
            moveHandle.Complete();
            
#if UNITY_EDITOR
            int index = m_debugData.BoidIndex;
            m_debugData.Position = m_transforms[index].position;
            m_debugData.Velocity = m_velocities[index];
            m_debugData.Steering = m_steerings[index];
#endif
        }

        private struct UpdateHashJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            // TODO: Look information about ParallelWriter.
            public NativeParallelMultiHashMap<int, int>.ParallelWriter SpatialHashMap;
            public float CellSize;
            
            public void Execute(int index)
            {
                int3 gridPosition = (int3)math.floor(Positions[index] / CellSize);
                int hash = GetCellHash(gridPosition);
                SpatialHashMap.Add(hash, index);
            }
        }

        private struct UpdateProbesJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            [ReadOnly]
            public NativeArray<float3> Velocities;
            [NativeDisableParallelForRestriction]
            public NativeArray<float3> Probes;
            public float ProbeLength;
            public float ProbeAngle;
            
            public void Execute(int index)
            {
                float3 originPosition = Positions[index];

                if (math.lengthsq(Velocities[index]) < 0) return;
                
                float length = math.length(Velocities[index]) * ProbeLength;
                float3 forward = math.normalize(Velocities[index]);

                if (index % PROBE_OFFSET == 0)
                {
                    Probes[index] = originPosition + forward * length;
                    return;
                }
                
                float3 right = math.normalize(math.cross(math.up(), forward));
                float3 up = math.cross(forward, right);

                float3 ray = forward * length;
                
                quaternion upTilt = quaternion.AxisAngle(right, ProbeAngle);
                Probes[index * PROBE_OFFSET + 1] = originPosition + math.mul(upTilt, ray);
                
                quaternion downTilt = quaternion.AxisAngle(right, -ProbeAngle);
                Probes[index * PROBE_OFFSET + 1] = originPosition + math.mul(downTilt, ray);
                
                quaternion rightTilt = quaternion.AxisAngle(up, ProbeAngle);
                Probes[index * PROBE_OFFSET + 1] = originPosition + math.mul(rightTilt, ray);

                quaternion leftTilt = quaternion.AxisAngle(up, -ProbeAngle);
                Probes[index * PROBE_OFFSET + 1] = originPosition + math.mul(leftTilt, ray);
                
            }
        }

        private struct FlockSteeringJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            [ReadOnly]
            public NativeArray<float3> Velocities;
            public NativeArray<float3> Steerings;
            [ReadOnly]
            public NativeParallelMultiHashMap<int, int> SpatialHashMap;
            public float CellSize;

            public float SeparationRadius;
            public float SeparationThreshold;
            public float SeparationWeight;
            public float CohesionRadius;
            public float CohesionThreshold;
            public float CohesionWeight;
            public float AlignmentRadius;
            public float AlignmentThreshold;
            public float AlignmentWeight;

            public float MaxSpeed;
            
            public void Execute(int index)
            {
                float3 steeringVector = new float3();
                float3 separationForce = new float3();
                float3 cohesionForce = new float3();
                int cohesionCount = 0;
                float3 alignmentForce = new float3();
                int alignmentCount = 0;

                int3 gridPosition = (int3)math.floor(Positions[index] / CellSize);
                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    int3 otherGridPosition = gridPosition + new int3(x, y, z);
                    int hash = GetCellHash(otherGridPosition);
                    
                    if (!SpatialHashMap.TryGetFirstValue(hash, out var other, out var iterator)) continue;
                    do
                    {
                        if (index == other) continue;
                        
                        float3 vectorToNeighbour = Positions[other] - Positions[index];
                        float3 vectorFromNeighbour = vectorToNeighbour * -1.0f;

                        float lenSq = math.lengthsq(vectorToNeighbour);
                        float velSq = math.lengthsq(Velocities[index]);
                        
                        float dot = 0;
                        if (velSq > 0 && lenSq > 0)
                        {
                            dot = math.dot(math.normalize(Velocities[index]), math.normalize(vectorToNeighbour));
                        }
                    
                        if (lenSq < SeparationRadius * SeparationRadius &&
                            dot > SeparationThreshold)
                        {
                            separationForce += vectorFromNeighbour / math.length(vectorFromNeighbour);
                        }

                        if (lenSq < CohesionRadius * CohesionRadius &&
                            dot > CohesionThreshold)
                        {
                            cohesionForce += Positions[other];
                            cohesionCount++;   
                        }

                        if (lenSq < AlignmentRadius * AlignmentRadius &&
                            dot > AlignmentThreshold)
                        {
                            alignmentForce += Velocities[other];
                            alignmentCount++;
                        }
                    } while (SpatialHashMap.TryGetNextValue(out other, ref iterator));

                }
            
                steeringVector += separationForce * SeparationWeight;

                if (cohesionCount > 0)
                {
                    cohesionForce = cohesionForce/cohesionCount - Positions[index];
                    steeringVector += cohesionForce * CohesionWeight;
                }

                if (alignmentCount > 0)
                {
                    alignmentForce /= alignmentCount;
                    steeringVector += alignmentForce * AlignmentWeight;
                }

                if (math.lengthsq(steeringVector) > 0)
                {
                    Steerings[index] = math.normalize(steeringVector) * MaxSpeed;
                }
                else
                {
                    Steerings[index] = float3.zero;
                }
            }
        }

        private struct ContainmentSteeringJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            [ReadOnly]
            public NativeArray<float3> Velocities;
            public NativeArray<float3> Steerings;
            [ReadOnly]
            public NativeArray<float3> Probes;

            public float WorldRadius;
            public float MaxSpeed;
            
            public void Execute(int index)
            {
                for (int i = 0; i < PROBE_OFFSET; i++)
                {
                    float3 probe = Probes[PROBE_OFFSET * index + i]; 
                    if (math.lengthsq(probe) < WorldRadius * WorldRadius) continue;

                    float3 directionToProbe = math.normalize(probe - Positions[index]);
                    float3 collisionPoint = GetCollisionPoint(Positions[index], directionToProbe, WorldRadius);

                    if (math.lengthsq(Velocities[index]) == 0.0f) return;

                    float3 collisionNormal = math.normalize(-collisionPoint);
                    float3 velocityNormal = math.normalize(-Velocities[index]);
                    float3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);
                
                    Steerings[index] += perpendicular * MaxSpeed;
                }
            }
            
            private float3 GetCollisionPoint(float3 origin, float3 rayDirection, float radius)
            {
                float a = 1;
                float b = 2.0f * math.dot(rayDirection, origin);
                float c = math.dot(origin, origin) - radius * radius;
                SolveQuadratic(a, b, c, out float t0, out float t1);

                return origin + rayDirection * t1;
            }
            
            private void SolveQuadratic(float a, float b, float c, out float t0, out float t1)
            {
                t0 = t1 = 0;
                
                float discriminant = b * b - 4 * a * c;

                if (discriminant < 0) return;
                
                if (discriminant == 0) 
                {
                    t0 = t1 = -0.5f * b / a;
                }
                else 
                {
                    float q = b > 0 ? 
                        -0.5f * (b + Mathf.Sqrt(discriminant)) : 
                        -0.5f * (b - Mathf.Sqrt(discriminant));
                    t0 = q / a;
                    t1 = c / q;
                }

                if (t0 > t1) (t0, t1) = (t1, t0);
            }

            float3 GetPerpendicular(float3 collisionNormal, float3 forwardNormal)
            {
                float3 dotVector = forwardNormal * math.dot(collisionNormal, forwardNormal);
                return math.normalize(collisionNormal - dotVector);
            }
        }

        private struct UpdateMovementJob : IJobParallelForTransform
        {
            public NativeArray<float3> Positions;
            public NativeArray<float3> Velocities;
            [ReadOnly]
            public NativeArray<float3> Steerings;

            public float MaxSpeed;
            public float DeltaTime;
            
            public void Execute(int index, TransformAccess transform)
            {
                Velocities[index] += Steerings[index] * DeltaTime;
                float velocitySq = math.lengthsq(Velocities[index]);

                if (velocitySq <= 0) return;
                
                float3 forward = math.normalize(Velocities[index]);
                if (velocitySq > MaxSpeed * MaxSpeed)
                {
                    Velocities[index] = forward * MaxSpeed;
                }
                
                Positions[index] += Velocities[index] * DeltaTime;
                transform.position = Positions[index];
                
                quaternion lookAt = quaternion.LookRotation(forward, math.up());
                transform.rotation = lookAt;
            }
        }
        
        private static int GetCellHash(int3 gridPosition)
        {
            unchecked
            {
                int hash = gridPosition.x * 73856093;
                hash ^= gridPosition.y * 19349663;
                hash ^= gridPosition.z * 83492791;
                return hash;
            }
        }

#region Development Methods
#if UNITY_EDITOR
        [Serializable]
        public struct DebugData
        {
            public bool DrawGizmos;
            public GizmosType GizmosType;
            public float PointRadius;
            
            [Space]
            public int BoidIndex;

            [NonSerialized]
            public float3 Position;
            [NonSerialized]
            public float3 Velocity;
            [NonSerialized]
            public float3 Steering;
        }

        [Serializable]
        public struct GizmosType
        {
            public bool WorldGizmo;
            public bool BoidGizmo;
        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            if (!m_debugData.GizmosType.WorldGizmo) return;
            DrawWorld();

            if (!m_debugData.GizmosType.BoidGizmo) return;
            DrawBoid();
        }

        private void DrawWorld()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.WorldRadius);
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.SpawnRadius);
        }

        private void DrawBoid()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity + m_debugData.Steering, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position + m_debugData.Velocity, m_debugData.Position + m_debugData.Velocity + m_debugData.Steering);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
        }
#endif
#endregion
    }
}