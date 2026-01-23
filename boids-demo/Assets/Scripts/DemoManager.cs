using System;
using Demo.Utilities;
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
            public int Count;
        }
        
        [Serializable]
        private struct BoidData
        {
            [Header("Move")]
            public float MinSpeed;
            public float MaxSpeed;
            
            [Header("Probes")]
            [Tooltip("Probe Length is calculated by multiplying the velocity vector by this multiplier.")]
            public float ProbeLengthMultiplier;
            [Tooltip("Probe Angle defines the tilt rotation of the four directional probes.")]
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
        
        // Data is stored in NativeArrays to ensure linear memory access, minimizing cache misses and allowing for Burst compilation.
        private TransformAccessArray m_transforms;
        private NativeArray<float3> m_positions;
        private NativeArray<quaternion> m_rotations;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_steerings;
        private NativeParallelMultiHashMap<int, int> m_spatialHashMap;
        private NativeArray<float3> m_probes;
        
        private JobHandle m_boidsHandle;

        // Spatial Hash Grid cell size is assigned to the highest steering radius to ensure the boids
        // query neighbours that are within their field of vision.
        private float m_gridCellSize;

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

        private void OnValidate()
        {
            // By setting the grid cell size to the maximum steering radius, we ensure that all potential boid
            // neighbours are located within the 27 adjacent cells.
            m_gridCellSize = math.max(m_boidData.SeparationRadius, m_boidData.CohesionRadius);
            m_gridCellSize = math.max(m_gridCellSize, m_boidData.AlignmentRadius);
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

        private void Update()
        {
            m_boidsHandle.Complete();
            
#if UNITY_EDITOR
            int index = m_debugData.BoidIndex;
            m_debugData.Position = m_transforms[index].position;
            m_debugData.Velocity = m_velocities[index];
            m_debugData.Steering = m_steerings[index];
            m_debugData.Probe0 = m_probes[PROBES_PER_BOID * index];
            m_debugData.Probe1 = m_probes[PROBES_PER_BOID * index + 1];
            m_debugData.Probe2 = m_probes[PROBES_PER_BOID * index + 2];
            m_debugData.Probe3 = m_probes[PROBES_PER_BOID * index + 3];
            m_debugData.Probe4 = m_probes[PROBES_PER_BOID * index + 4];
#endif
            // Rebuilds the spatial hash grid with the boids position.
            m_spatialHashMap.Clear();
            UpdateSpatialHashGridJob spatialHashGridJob = new UpdateSpatialHashGridJob
            {
                Positions = m_positions,
                SpatialHashMap = m_spatialHashMap.AsParallelWriter(),
                WorldSize = m_worldData.WorldRadius,
                CellSize = m_gridCellSize
            };
            JobHandle hashHandle = spatialHashGridJob.Schedule(m_worldData.Count, 64);
            
            // Updates position and rotation of probes based on boid position and velocity.
            UpdateProbesJob probesJob = new UpdateProbesJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                ProbeLength = m_boidData.ProbeLengthMultiplier,
                ProbeAngle = m_boidData.ProbeAngle
            };
            JobHandle probesHandle = probesJob.Schedule(m_worldData.Count, 64);
            
            JobHandle setupHandle = JobHandle.CombineDependencies(hashHandle, probesHandle);
            
            // Overrides the steering vector to the calculate value (steering = value), because
            // steering behaviours are only accumulative for one frame.
            FlockSteeringJob flockJob = new FlockSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings,
                SpatialHashMap = m_spatialHashMap,
                WorldSize = m_worldData.WorldRadius,
                CellSize = m_gridCellSize,
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
            
            // Containment is calculated after flocking to ensure we know where the boid is headed. This increments
            // to the steering vector by the calculated value (steering += value).
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
            
            // Calculates velocity based on the previously calculated steering vector and update boids position.
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
        /// Using a <c>ParallelMultiHashMap</c> for a spatial hash grid allowed me to reduce the query to O(n).
        /// Without the spatial hash grid the complexity would rise to O(n^2).
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
        /// Probes are rays intended as a "vision cone" for each boid; they are needed to check for collisions.
        /// These probes act similarly to the built-in Raycasts, but avoid the performance overhead of the
        /// physics system.
        /// </summary>
        /// <param name="ProbeAngle">The angle of rotation for the tilted probes.</param>
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
                float length = CustomMath.Length(Velocities[index]) * ProbeLength;
                float3 forward = CustomMath.Normalize(Velocities[index]);
                float3 ray = forward * length;

                float3 globalUp = math.up();
                float3 cross = math.cross(globalUp, forward);
                float3 right = CustomMath.Normalize(cross);
                float3 up = math.cross(forward, right);
                
                // Each boid has five probes: the first follows the direction of the velocity.
                Probes[index * PROBES_PER_BOID] = originPosition + ray;
                
                // The remaining four are tilted upwards, downwards, leftwards and rightwards.
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

                float3 velocityNormalized = CustomMath.Normalize(Velocities[index]);
                float3 division = CustomMath.Divide(Positions[index], CellSize);
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
                        
                        float dot = math.dot(velocityNormalized, CustomMath.Normalize(vectorToNeighbour));
                    
                        if (distanceSqToNeighbour < SeparationRadius * SeparationRadius && dot > SeparationThreshold)
                        {
                            float distanceToNeighbour = math.sqrt(distanceSqToNeighbour);
                            separationForce += CustomMath.Divide(-vectorToNeighbour, distanceToNeighbour);
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

                Steerings[index] = CustomMath.Normalize(steeringVector) * MaxSpeed;
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

                    float3 probeDirection = CustomMath.Normalize(probe - Positions[index]);
                    float3 collisionPoint = GetCollisionPoint(Positions[index], probeDirection, WorldRadius);

                    float3 collisionNormal = CustomMath.Normalize(-collisionPoint);
                    float3 velocityNormal = CustomMath.Normalize(-Velocities[index]);
                    float3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);
                
                    Steerings[index] += perpendicular * MaxSpeed;
                    return;
                }
            }

            /// <summary>
            /// We simplified the Analytic Solution presented by Jean-Colas Prunier. This simplification outputs
            /// the correct position of a collision only when the ray origin is contained within the boundary radius.
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
            private void SolveQuadratic(float a, float b, float c, out float t)
            {
                t = 0;
                
                float discriminant = b * b - 4 * a * c;

                float q = b > 0 ? 
                    -0.5f * (b + Mathf.Sqrt(discriminant)) : 
                    -0.5f * (b - Mathf.Sqrt(discriminant));
                
                t = c / q;
            }

            /// <summary>
            /// Returning a perpendicular force allows for corrective lateral steering with minimal deceleration.
            /// Other behaviours, like flee, would cause the boid to slow down and steer perpendicular to the boundary.
            /// </summary>
            /// <returns>
            /// Returns the steering direction, perpendicular to the velocity, required to steer away from the boundary.
            /// </returns>
            float3 GetPerpendicular(float3 collisionNormal, float3 forwardNormal)
            {
                float3 dotVector = forwardNormal * math.dot(collisionNormal, forwardNormal);
                return CustomMath.Normalize(collisionNormal - dotVector);
            }
        }

        /// <summary>
        /// Apply the steering vector to the boids velocity and clamp the velocity to the <c>BoidData.MaxSpeed</c> to
        /// prevent infinite acceleration. Finally, update their rotation to "look at" the new velocity direction.
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
                float3 forward = CustomMath.Normalize(Velocities[index]);
                if (velocitySq > MaxSpeed * MaxSpeed)
                    Velocities[index] = forward * MaxSpeed;
                
                Positions[index] += Velocities[index] * DeltaTime;
                transform.position = Positions[index];
                
                Rotations[index] = CustomMath.LookRotation(forward, math.up());
                transform.rotation = Rotations[index];
            }
        }
        
        private static int GetIndex(int3 gridPosition, int gridSize)
        {
            return gridPosition.x + gridPosition.y * gridSize + gridPosition.z * gridSize;
        }
        
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

            [NonSerialized]
            public float3 Probe0;
            [NonSerialized]
            public float3 Probe1;
            [NonSerialized]
            public float3 Probe2;
            [NonSerialized]
            public float3 Probe3;
            [NonSerialized]
            public float3 Probe4;
        }

        /// <summary>
        /// Struct with booleans to filter draw calls of the Gizmos.
        /// </summary>
        [Serializable]
        public struct GizmosType
        {
            public bool WorldGizmo;
            public bool GridGizmo;
            public bool SteeringGizmos;
            public bool VelocityGizmos;
            public bool ProbesGizmos;
        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            if (m_debugData.GizmosType.WorldGizmo) DrawWorld();
            if (m_debugData.GizmosType.GridGizmo) DrawGrid();
            if (m_debugData.GizmosType.SteeringGizmos) DrawSteering();
            if (m_debugData.GizmosType.VelocityGizmos) DrawVelocity();
            if (m_debugData.GizmosType.ProbesGizmos) DrawProbes();
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
            int gridRadius = (int)(m_worldData.WorldRadius / m_gridCellSize);
            for (int x = -gridRadius; x < gridRadius; x++)
            for (int y = -gridRadius; y < gridRadius; y++)
            for (int z = -gridRadius; z < gridRadius; z++)
            {
                float3 gridPosition = new float3(x, y, z);
                float3 worldPosition = gridPosition * m_gridCellSize;
                if (math.lengthsq(worldPosition) > m_worldData.WorldRadius * m_worldData.WorldRadius) continue;
                Gizmos.DrawWireCube(worldPosition, Vector3.one * m_gridCellSize);
            }
        }

        private void DrawSteering()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity + m_debugData.Steering, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position + m_debugData.Velocity, m_debugData.Position + m_debugData.Velocity + m_debugData.Steering);
        }

        private void DrawVelocity()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
        }

        private void DrawProbes()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe0);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe1);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe2);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe3);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe4);
        }
#endif
#endregion
    }
}