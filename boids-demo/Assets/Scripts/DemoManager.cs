using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
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
        private NativeArray<quaternion> m_rotations;
        private NativeArray<float3> m_positions;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_steerings;
        private NativeArray<float3> m_probes;

        // Spatial Hash Grid cell size is assigned to the highest steering radius to ensure the boids query neighbours
        // that are within their field of vision.
        private NativeParallelMultiHashMap<uint, int> m_spatialGrid;
        private float m_cellSize;

        private JobHandle m_boidsHandle;
        
        private const int PROBES_PER_BOID = 5;
    
        private void Awake()
        {
            m_rotations = new NativeArray<quaternion>(m_worldData.Count, Allocator.Persistent);
            m_positions = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_steerings = new NativeArray<float3>(m_worldData.Count, Allocator.Persistent);
            m_probes = new NativeArray<float3>(m_worldData.Count * PROBES_PER_BOID, Allocator.Persistent);

            m_spatialGrid = new NativeParallelMultiHashMap<uint, int>(m_worldData.Count, Allocator.Persistent);
        }

        private void Start()
        {
            InstantiateBoids();

            // By setting the grid cell size to the maximum steering radius, we ensure that all potential boid neighbours
            // are located within the 27 adjacent cells.
            m_cellSize = Mathf.Max(m_boidData.SeparationRadius, m_boidData.CohesionRadius, m_boidData.AlignmentRadius);
            
            Cursor.lockState = CursorLockMode.Locked;
        }
        
        private void OnDestroy()
        {
            m_boidsHandle.Complete();
            
            m_transforms.Dispose();
            m_positions.Dispose();
            m_rotations.Dispose();
            m_velocities.Dispose();
            m_steerings.Dispose();
            m_probes.Dispose(); 
            
            m_spatialGrid.Dispose();
        }
        
        private void InstantiateBoids()
        {
            Transform[] transforms = new Transform[m_worldData.Count];
            for (int index = 0; index < m_worldData.Count; index++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * m_worldData.SpawnRadius;
                Quaternion randomRotation = Random.rotationUniform;
                
                GameObject boid = Instantiate(m_prefab, randomPosition, randomRotation);
                boid.name = $"Boids_{index}";
                
                float speed = Random.Range(m_boidData.MinSpeed, m_boidData.MaxSpeed);
                Vector3 randomVelocity = boid.transform.forward * speed;

                transforms[index] = boid.transform;
                m_positions[index] = randomPosition;
                m_rotations[index] = randomRotation;
                m_velocities[index] = randomVelocity;
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
            m_spatialGrid.Clear();
            var spatialGridJob = new SpatialGridJob
            {
                Positions = m_positions,
                SpatialGrid = m_spatialGrid.AsParallelWriter(),
                CellSize = m_cellSize
            };
            JobHandle spatialGridHandle = spatialGridJob.Schedule(m_worldData.Count, 64);

            // Updates position and rotation of probes based on boid position and velocity.
            var probesJob = new ProbesJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                ProbeLengthMultiplier = m_boidData.ProbeLengthMultiplier,
                ProbeAngle = m_boidData.ProbeAngle
            };
            JobHandle probeHandle = probesJob.Schedule(m_worldData.Count, 64);
            
            JobHandle setupHandle = JobHandle.CombineDependencies(spatialGridHandle, probeHandle);

            // Overrides the steering vector to the calculate value (steering = value), because steering behaviours are
            // only accumulative for one frame.
            var flockSteeringJob = new FlockSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                SpatialGrid = m_spatialGrid,
                Steerings = m_steerings,
                CellSize = m_cellSize,
                SeparationRadius = m_boidData.SeparationRadius,
                SeparationDot = m_boidData.SeparationDot,
                SeparationWeight = m_boidData.SeparationWeight,
                CohesionRadius = m_boidData.CohesionRadius,
                CohesionDot = m_boidData.CohesionDot,
                CohesionWeight = m_boidData.CohesionWeight,
                AlignmentRadius = m_boidData.AlignmentRadius,
                AlignmentDot = m_boidData.AlignmentDot,
                AlignmentWeight = m_boidData.AlignmentWeight,
                MaxSpeed = m_boidData.MaxSpeed
            };
            JobHandle flockingSteeringHandle = flockSteeringJob.Schedule(m_worldData.Count, 64, setupHandle);

            // Containment is calculated after flocking to ensure we know where the boid is headed. This increments
            // to the steering vector by the calculated value (steering += value).
            var containmentSteeringJob = new ContainmentSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                Steerings = m_steerings,
                WorldRadius = m_worldData.WorldRadius,
                MaxSpeed = m_boidData.MaxSpeed
            };
            JobHandle containmentSteeringHandle = containmentSteeringJob.Schedule(m_worldData.Count, 64, flockingSteeringHandle);

            // Calculates velocity based on the previously calculated steering vector and update boids position.
            var movementJob = new MovementJob
            {
                Steerings = m_steerings,
                Rotations = m_rotations,
                Positions = m_positions,
                Velocities = m_velocities,
                DeltaTime = Time.deltaTime,
                MaxSpeed = m_boidData.MaxSpeed
            };
            m_boidsHandle = movementJob.Schedule(m_transforms, containmentSteeringHandle);
        }
        
        /// <summary>
        /// Using a <c>ParallelMultiHashMap</c> for a spatial hash grid allowed me to reduce the query to O(n).
        /// Without the spatial hash grid the complexity would rise to O(n^2).
        /// </summary>
        [BurstCompile]
        private struct SpatialGridJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Positions;

            public NativeParallelMultiHashMap<uint, int>.ParallelWriter SpatialGrid;
            public float CellSize;
            
            public void Execute(int index)
            {
                int3 gridPosition = (int3)math.floor(Positions[index] / CellSize);
                uint key = GetHash(gridPosition);
                SpatialGrid.Add(key, index);
            }
        }
        
        /// <summary>
        /// Probes are rays intended as a "vision cone" for each boid; they are needed to check for collisions.
        /// These probes act similarly to the built-in Raycasts, but avoid the performance overhead of the
        /// physics system.
        /// </summary>
        [BurstCompile]
        private struct ProbesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<float3> Velocities;
            
            [NativeDisableParallelForRestriction]
            public NativeArray<float3> Probes;
            
            public float ProbeLengthMultiplier;
            public float ProbeAngle;
            
            // Because each boid has five probes, the probes array length is 'WorldData.Count * PROBES_PER_BOID'.
            // We use 'PROBES_PER_BOID * index + n' where n = { 0..4 } to access the five probes associated with the boid index.
            public void Execute(int index)
            {
                float3 originPosition = Positions[index];
                float length = math.length(Velocities[index]) * ProbeLengthMultiplier;
                float3 forward = math.normalizesafe(Velocities[index]);
                float3 ray = forward * length;

                float3 globalUp = math.up();
                float3 cross = math.cross(globalUp, forward);
                float3 right = math.normalizesafe(cross);
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
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<float3> Velocities;
            [ReadOnly] public NativeParallelMultiHashMap<uint, int> SpatialGrid;

            public NativeArray<float3> Steerings;
            public float CellSize;
            public float SeparationRadius;
            public float SeparationDot;
            public float SeparationWeight;
            public float CohesionRadius;
            public float CohesionDot;
            public float CohesionWeight;
            public float AlignmentRadius;
            public float AlignmentDot;
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

                int3 myGridPosition = (int3)math.floor(Positions[index] / CellSize);
                
                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    int3 otherGridPosition = myGridPosition + new int3(x, y, z);
                    uint hash = GetHash(otherGridPosition);

                    bool hasHashList = SpatialGrid.TryGetFirstValue(hash, out var other, out var iterator);
                    if (!hasHashList) continue;

                    do
                    {
                        if (index == other) continue;

                        float3 vectorToNeighbour = Positions[other] - Positions[index];
                        float distanceSqToNeighbour = math.lengthsq(vectorToNeighbour);

                        float dot = math.dot(
                            math.normalizesafe(Velocities[index]),
                            math.normalize(vectorToNeighbour));
                        
                        if (distanceSqToNeighbour < SeparationRadius * SeparationRadius && dot > SeparationDot)
                        {
                            float distanceToNeighbour = math.sqrt(distanceSqToNeighbour);
                            separationForce += -vectorToNeighbour / distanceToNeighbour;
                        }

                        if (distanceSqToNeighbour < CohesionRadius * CohesionRadius && dot > CohesionDot)
                        {
                            cohesionForce += Positions[other];
                            cohesionCount++;
                        }

                        if (distanceSqToNeighbour < AlignmentRadius * AlignmentRadius && dot > AlignmentDot)
                        {
                            alignmentForce += Velocities[other];
                            alignmentCount++;
                        }
                    } while (SpatialGrid.TryGetNextValue(out other, ref iterator));
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

                Steerings[index] = math.normalizesafe(steeringVector) * MaxSpeed;
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
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<float3> Velocities;
            [ReadOnly] public NativeArray<float3> Probes;

            public NativeArray<float3> Steerings;
            public float WorldRadius;
            public float MaxSpeed;
            
            public void Execute(int index)
            {
                float3 steering = new float3(0.0f);
                
                for (int i = 0; i < PROBES_PER_BOID; i++)
                {
                    float3 probe = Probes[index * PROBES_PER_BOID + i];
                    if (math.lengthsq(probe) < WorldRadius * WorldRadius) continue;

                    float3 probeDirection = math.normalize(probe - Positions[index]);
                    float3 collisionPoint = GetCollisionPoint(Positions[index], probeDirection, WorldRadius);

                    float3 collisionNormal = math.normalize(-collisionPoint);
                    float3 velocityNormal = math.normalizesafe(-Velocities[index]);
                    float3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);
            
                    steering += perpendicular;
                }

                Steerings[index] += math.normalizesafe(steering) * MaxSpeed;
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
                float t = SolveQuadratic(a, b, c);

                return rayOrigin + rayDirection * t;
            }

            /// <returns>Distance between Ray Origin and Collision Point.</returns>
            private float SolveQuadratic(float a, float b, float c)
            {
                float discriminant = b * b - 4 * a * c;

                float q = b > 0 ? 
                    -0.5f * (b + math.sqrt(discriminant)) : 
                    -0.5f * (b - math.sqrt(discriminant));
                
                return c / q;
            } 
            
            /// <summary>
            /// Returning a perpendicular force allows for corrective lateral steering with minimal deceleration.
            /// Other behaviours, like flee, would cause the boid to slow down and steer perpendicular to the boundary.
            /// </summary>
            /// <returns> Returns the steering direction, perpendicular to the velocity, required to steer away from the boundary.</returns>
            private float3 GetPerpendicular(float3 collisionNormal, float3 forwardNormal)
            {
                float3 dotVector = forwardNormal * math.dot(collisionNormal, forwardNormal);
                return math.normalize(collisionNormal - dotVector);
            }
        }

        /// <summary>
        /// Apply the steering vector to the boids velocity and clamp the velocity to the <c>BoidData.MaxSpeed</c> to
        /// prevent infinite acceleration. Finally, update their rotation to "look at" the new velocity direction.
        /// </summary>
        [BurstCompile]
        private struct MovementJob : IJobParallelForTransform
        {
            [ReadOnly] public NativeArray<float3> Steerings;

            public NativeArray<quaternion> Rotations;
            public NativeArray<float3> Positions;
            public NativeArray<float3> Velocities;

            public float DeltaTime;
            public float MaxSpeed;
            
            public void Execute(int index, TransformAccess transform)
            {
                Velocities[index] += Steerings[index] * DeltaTime;

                float velocitySq = math.lengthsq(Velocities[index]);
                float3 forward = math.normalizesafe(Velocities[index]);
                if (velocitySq > MaxSpeed * MaxSpeed)
                    Velocities[index] = forward * MaxSpeed;
            
                Positions[index] += Velocities[index] * DeltaTime;
                transform.position = Positions[index];
            
                Rotations[index] = quaternion.LookRotation(forward, math.up());
                transform.rotation = Rotations[index];
            }
        }
        
        private static uint GetHash(int3 gridPosition)
        {
            return math.hash(gridPosition);
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
            
            [Space]
            public float PointRadius;

            [Space]
            public int BoidIndex;

            [NonSerialized] public float3 Position;
            [NonSerialized] public float3 Velocity;
            [NonSerialized] public float3 Steering;
            [NonSerialized] public float3 Probe0;
            [NonSerialized] public float3 Probe1;
            [NonSerialized] public float3 Probe2;
            [NonSerialized] public float3 Probe3;
            [NonSerialized] public float3 Probe4;
        }

        /// <summary>
        /// Struct with booleans to filter draw calls of the Gizmos.
        /// </summary>
        [Serializable]
        public struct GizmosType
        {
            public bool WorldGizmo;
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
            if (m_debugData.GizmosType.SteeringGizmos) DrawSteering();
            if (m_debugData.GizmosType.VelocityGizmos) DrawVelocity();
            if (m_debugData.GizmosType.ProbesGizmos) DrawProbes();
        }

        private void DrawWorld()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(float3.zero, m_worldData.WorldRadius);
            Gizmos.DrawWireSphere(float3.zero, m_worldData.SpawnRadius);
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