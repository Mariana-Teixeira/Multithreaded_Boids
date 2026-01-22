using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;
using UnityEngine.Jobs;
using Random = UnityEngine.Random;

namespace Demo.Boids
{
    [BurstCompile]
    public class DemoManager : MonoBehaviour
    {
        [Serializable]
        private struct WorldData
        {
            public float WorldRadius;
            public float SpawnRadius;
            public float CellSize;
            public int Count;
        }
        
        [Serializable]
        private struct BoidData
        {
            [Header("Move")]
            public float MinSpeed;
            public float MaxSpeed;
            
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
        private NativeArray<quaternion> m_rotations;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_steerings;
        private NativeParallelMultiHashMap<int, int> m_spatialHashMap;
        private NativeArray<float3> m_probes;

        private JobHandle m_boidsHandle;

        private const int PROBES_PER_BOID = 5;
    
        private void Awake()
        {
            m_positions = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_rotations = new NativeArray<quaternion>(m_worldData.Count, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_steerings = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_spatialHashMap = new NativeParallelMultiHashMap<int, int>(m_worldData.Count, Allocator.Persistent);
            m_probes = new NativeArray<float3>(m_worldData.Count * PROBES_PER_BOID, Allocator.Persistent);
        }

        private void OnDestroy()
        {
            m_boidsHandle.Complete();
            
            m_transforms.Dispose();
            m_positions.Dispose();
            m_rotations.Dispose();
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

        /// <summary>
        /// Schedules the Unity Jobs responsible for updating the Spatial Hash Grid, Probes and Steering Behaviours.
        /// </summary>
        private void Update()
        {
            m_boidsHandle.Complete();
            
#if UNITY_EDITOR
            int index = m_debugData.BoidIndex;
            m_debugData.Position = m_transforms[index].position;
            m_debugData.Velocity = m_velocities[index];
            m_debugData.Steering = m_steerings[index];
#endif
            
            // Update Boids position on the Spatial Hash Grid.
            m_spatialHashMap.Clear();
            UpdateSpatialHashGridJob spatialHashGridJob = new UpdateSpatialHashGridJob
            {
                Positions = m_positions,
                SpatialHashMap = m_spatialHashMap.AsParallelWriter(),
                WorldSize = m_worldData.WorldRadius,
                CellSize = m_worldData.CellSize
            };
            JobHandle hashHandle = spatialHashGridJob.Schedule(m_worldData.Count, 64);
            
            // Update Probe Position/Rotation for Boids.
            // Probes are the Boids vision; they are used to check for collisions.
            UpdateProbesJob probesJob = new UpdateProbesJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                ProbeLength = m_boidData.ProbeLength,
                ProbeAngle = m_boidData.ProbeAngle
            };
            JobHandle probesHandle = probesJob.Schedule(m_worldData.Count, 64);
            
            JobHandle setupHandle = JobHandle.CombineDependencies(hashHandle, probesHandle);
            
            // Updates the steering vector according to the separation, cohesion and alignment principles.
            // Overrides the previous frames steering vector to the calculate value (steering = value).
            FlockSteeringJob flockJob = new FlockSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings,
                SpatialHashMap = m_spatialHashMap,
                WorldSize = m_worldData.WorldRadius,
                CellSize = m_worldData.CellSize,
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
            
            // Updates the steering vector according to the containment principle (similar to obstacle avoidance).
            // Increments the calculated value to the previous steering vector (steering += value).
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
            
            // Calculates velocity based on the previously calculated steering vector.
            // Moves boids using the TransformAccessArray (position += velocity).
            UpdateMovementJob movementJob = new UpdateMovementJob
            {
                Positions = m_positions,
                Rotations = m_rotations,
                Velocities = m_velocities,
                Steerings = m_steerings,
                MaxSpeed = m_boidData.MaxSpeed,
                DeltaTime = Time.deltaTime
            };
            m_boidsHandle = movementJob.Schedule(m_transforms, containmentHandle);
        }
        
        /// <summary>
        /// Each Boid updates their index position in the spatial hash grid.
        /// </summary>
        [BurstCompile]
        private struct UpdateSpatialHashGridJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            public NativeParallelMultiHashMap<int, int>.ParallelWriter SpatialHashMap;
            public float WorldSize;
            public float CellSize;
            
            public void Execute(int index)
            {
                int3 gridPosition = (int3)math.floor(Positions[index] / CellSize);
                int gridSize = (int)(WorldSize / CellSize);
                int key = GetIndex(gridPosition, gridSize);
                SpatialHashMap.Add(key, index);
            }
        }
        
        /// <summary>
        /// Each boid has five associated probes: one probe follows the direction of the velocity, the remaining four
        /// probes are rotated upwards, downwards, leftwards and rightwards from that initial direction.
        /// </summary>
        /// <param name="ProbeAngle">The angle of rotation for the rotated probes.</param>
        [BurstCompile]
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
            
            // Because each boid has five probes, the probes array length is 'WorldData.Count * PROBES_PER_BOID'.
            // We use 'PROBES_PER_BOID * index + n' where n = { 0..4 } to access the five probes associated with the boid index.
            public void Execute(int index)
            {
                float3 originPosition = Positions[index];
                float length = SafeLength(Velocities[index]) * ProbeLength;
                float3 forward = SafeNormalize(Velocities[index]);
                float3 ray = forward * length;

                float3 globalUp = math.up();
                float3 cross = math.cross(globalUp, forward);
                float3 right = SafeNormalize(cross);
                float3 up = math.cross(forward, right);
                
                Probes[index * PROBES_PER_BOID] = originPosition + ray;

                quaternion upTilt = quaternion.AxisAngle(right, ProbeAngle);
                Probes[index * PROBES_PER_BOID + 1] = originPosition + math.mul(upTilt, ray);

                quaternion downTilt = quaternion.AxisAngle(right, -ProbeAngle);
                Probes[index * PROBES_PER_BOID + 2] = originPosition + math.mul(downTilt, ray);

                quaternion rightTilt = quaternion.AxisAngle(up, ProbeAngle);
                Probes[index * PROBES_PER_BOID + 3] = originPosition + math.mul(rightTilt, ray);

                quaternion leftTilt = quaternion.AxisAngle(up, -ProbeAngle);
                Probes[index * PROBES_PER_BOID + 4] = originPosition + math.mul(leftTilt, ray);
            }
        }

        /// <summary>
        /// Follows a Flock Steering Behaviour by combining Separation, Cohesion and Alignment Behaviours.
        /// Uses the Spatial Hash Grid to get neighbouring boids of adjacent cells.
        /// </summary>
        /// <a href="https://www.red3d.com/cwr/steer/gdc99/">Steering Behaviors For Autonomous Characters</a>
        [BurstCompile]
        private struct FlockSteeringJob : IJobParallelFor
        {
            [ReadOnly]
            public NativeArray<float3> Positions;
            [ReadOnly]
            public NativeArray<float3> Velocities;
            public NativeArray<float3> Steerings;
            [ReadOnly]
            public NativeParallelMultiHashMap<int, int> SpatialHashMap;
            public float WorldSize;
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
                float3 alignmentForce = new float3();
                
                int cohesionCount = 0;
                int alignmentCount = 0;

                float3 velocityNormalized = SafeNormalize(Velocities[index]);
                float3 division = SafeDivide(Positions[index], CellSize);
                int3 gridPosition = (int3)math.floor(division);

                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    int3 otherGridPosition = gridPosition + new int3(x, y, z);
                    int gridSize = (int)(WorldSize / CellSize);
                    int hash = GetIndex(otherGridPosition, gridSize);

                    bool fetchValue = SpatialHashMap.TryGetFirstValue(hash, out var other, out var iterator);
                    if (!fetchValue) continue;
                    
                    do
                    {
                        if (index == other) continue;
                        
                        float3 vectorToNeighbour = Positions[other] - Positions[index];
                        float distanceSqToNeighbour = math.lengthsq(vectorToNeighbour);
                        
                        float dot = math.dot(velocityNormalized, SafeNormalize(vectorToNeighbour));
                    
                        if (distanceSqToNeighbour < SeparationRadius * SeparationRadius && dot > SeparationThreshold)
                        {
                            float distanceToNeighbour = math.sqrt(distanceSqToNeighbour);
                            separationForce += SafeDivide(-vectorToNeighbour, distanceToNeighbour);
                        }
                        
                        if (distanceSqToNeighbour < CohesionRadius * CohesionRadius && dot > CohesionThreshold)
                        {
                            cohesionForce += Positions[other];
                            cohesionCount++;   
                        }

                        if (distanceSqToNeighbour < AlignmentRadius * AlignmentRadius && dot > AlignmentThreshold)
                        {
                            alignmentForce += Velocities[other];
                            alignmentCount++;
                        }
                    } while (SpatialHashMap.TryGetNextValue(out other, ref iterator));
                }
            
                steeringVector += separationForce * SeparationWeight;

                if (cohesionCount > 0)
                {
                    cohesionForce = cohesionForce / cohesionCount - Positions[index];
                    steeringVector += cohesionForce * CohesionWeight;
                }

                if (alignmentCount > 0)
                {
                    alignmentForce = alignmentForce / alignmentCount;
                    steeringVector += alignmentForce * AlignmentWeight;
                }

                Steerings[index] = SafeNormalize(steeringVector) * MaxSpeed;
            }
        }

        /// <summary>
        /// We limit the boids movement to the <c>WorldData.Radius</c> by using object avoidance steering behaviour.
        /// Since the boids are contained inside a sphere, we use Ray-Sphere Intersection and Probes to calculate when
        /// and how a boid needs to redirect their steering.
        /// </summary>
        /// <a href="https://www.red3d.com/cwr/steer/gdc99/">Steering Behaviors For Autonomous Characters</a>
        [BurstCompile]
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
                for (int i = 0; i < PROBES_PER_BOID; i++)
                {
                    float3 probe = Probes[index * PROBES_PER_BOID + i];
                    if (math.lengthsq(probe) < WorldRadius * WorldRadius) continue;

                    float3 probeDirection = SafeNormalize(probe - Positions[index]);
                    float3 collisionPoint = GetCollisionPoint(Positions[index], probeDirection, WorldRadius);

                    float3 collisionNormal = SafeNormalize(-collisionPoint);
                    float3 velocityNormal = SafeNormalize(-Velocities[index]);
                    float3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);
                
                    Steerings[index] += perpendicular * MaxSpeed;
                    return;
                }
            }

            /// <summary>
            /// We use a simplified the Analytic Solution presented by Jean-Colas Prunier. This simplification outputs
            /// the correct position of a collision only when the ray origin is contained within the collider radius.
            /// </summary>
            /// <a href="https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html">Ray-Sphere Intersection</a>
            private float3 GetCollisionPoint(float3 rayOrigin, float3 rayDirection, float colliderRadius)
            {
                float a = 1;
                float b = 2.0f * math.dot(rayDirection, rayOrigin);
                float c = math.dot(rayOrigin, rayOrigin) - colliderRadius * colliderRadius;
                SolveQuadratic(a, b, c, out float t);

                return rayOrigin + rayDirection * t;
            }


            /// <param name="t">Distance between Ray Origin and Collision Point.</param>
            /// <a href="https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html">Ray-Sphere Intersection</a>
            private void SolveQuadratic(float a, float b, float c, out float t)
            {
                t = 0;
                
                float discriminant = b * b - 4 * a * c;

                float q = b > 0 ? 
                    -0.5f * (b + Mathf.Sqrt(discriminant)) : 
                    -0.5f * (b - Mathf.Sqrt(discriminant));
                
                t = c / q;
            }

            float3 GetPerpendicular(float3 collisionNormal, float3 forwardNormal)
            {
                float3 dotVector = forwardNormal * math.dot(collisionNormal, forwardNormal);
                return SafeNormalize(collisionNormal - dotVector);
            }
        }

        /// <summary>
        /// Updates the velocity of each boid by their steering vector and applies it to position and rotation.
        /// Boids use their transform component to translate and rotate.
        /// </summary>
        [BurstCompile]
        private struct UpdateMovementJob : IJobParallelForTransform
        {
            public NativeArray<float3> Positions;
            public NativeArray<quaternion> Rotations;
            public NativeArray<float3> Velocities;
            [ReadOnly]
            public NativeArray<float3> Steerings;

            public float MaxSpeed;
            public float DeltaTime;
            
            public void Execute(int index, TransformAccess transform)
            {
                Velocities[index] += Steerings[index] * DeltaTime;

                float velocitySq = math.lengthsq(Velocities[index]);
                float3 forward = SafeNormalize(Velocities[index]);
                if (velocitySq > MaxSpeed * MaxSpeed)
                    Velocities[index] = forward * MaxSpeed;
                
                Positions[index] += Velocities[index] * DeltaTime;
                transform.position = Positions[index];
                
                Rotations[index] = SafeLookRotation(forward, math.up());
                transform.rotation = Rotations[index];
            }
        }
        
#region Math Helpers
        private static int GetIndex(int3 gridPosition, int gridSize)
        {
            return gridPosition.x + gridPosition.y * gridSize + gridPosition.z * gridSize;
        }

        /// <returns>Returns a zero when attempting to normalize a vector zero.</returns>
        private static float3 SafeNormalize(float3 vector)
        {
            return math.lengthsq(vector) > math.EPSILON ? math.normalize(vector) : float3.zero;
        }
        
        /// <returns>Returns a zero when attempting to get length of a vector zero.</returns>
        private static float SafeLength(float3 vector)
        {
            return math.lengthsq(vector) > math.EPSILON ? math.length(vector) : 0.0f;
        }

        /// <returns>Returns a zero when attempting to divide by zero.</returns>
        private static float3 SafeDivide(float3 numerator, float denominator)
        {
            return math.abs(denominator) < math.EPSILON ? float3.zero : numerator / denominator;
        }

        /// <returns>Returns quaternion identity when forward is zero vector.</returns>
        private static quaternion SafeLookRotation(float3 forward, float3 up)
        {
            return math.lengthsq(forward) < math.EPSILON ? quaternion.identity : quaternion.LookRotation(forward, up);
        }
#endregion

#region Debugging Methods
#if UNITY_EDITOR
        /// <summary>
        /// Struct containing all data necessary for the Gizmos.
        /// </summary>
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

        /// <summary>
        /// Struct with booleans to filter draw calls of the Gizmos.
        /// </summary>
        [Serializable]
        public struct GizmosType
        {
            public bool WorldGizmo;
            public bool GridGizmo;
            public bool BoidGizmo;
        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            if (m_debugData.GizmosType.WorldGizmo)
            {
                DrawWorld();
            }

            if (m_debugData.GizmosType.GridGizmo)
            {
                DrawGrid();
            }

            if (m_debugData.GizmosType.BoidGizmo)
            {
                DrawBoid();
            }
        }

        private void DrawWorld()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.WorldRadius);
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.SpawnRadius);
        }

        private void DrawGrid()
        {
            Gizmos.color = Color.white;
            int gridRadius = (int)(m_worldData.WorldRadius / m_worldData.CellSize);
            for (int x = -gridRadius; x < gridRadius; x++)
            for (int y = -gridRadius; y < gridRadius; y++)
            for (int z = -gridRadius; z < gridRadius; z++)
            {
                float3 gridPosition = new float3(x, y, z);
                float3 worldPosition = gridPosition * m_worldData.CellSize;
                if (math.lengthsq(worldPosition) > m_worldData.WorldRadius * m_worldData.WorldRadius) continue;
                Gizmos.DrawWireCube(worldPosition, Vector3.one * m_worldData.CellSize);
            }
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