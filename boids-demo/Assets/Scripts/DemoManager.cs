using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.UI;
using Random = UnityEngine.Random;

namespace Demo.Boids
{
    public class DemoManager : MonoBehaviour
    {
        [Header("Data")]
        [SerializeField] private WorldData m_worldData;
        [SerializeField] private BoidData m_boidData;
        
        [Header("Debug")]
        [SerializeField] private bool m_spawnDebugOnStart;
        
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
        
        private GameObject m_boidParent;

        private DemoDebug m_demoDebug;
        
        private bool m_useSpatialGrid;
        private int m_perceivedBoidsPerCell;
        private float m_spawnRadius;
        private int m_count;

        private void Awake()
        {
            m_perceivedBoidsPerCell = DemoData.DEFAULT_CELL_MULTIPLIER * DemoData.MAX_CELL_COUNT;
            m_spawnRadius = m_worldData.DefaultSpawnRadius;
            m_count = m_worldData.GetCount();
            
            ResetVariables();
        }

        private void Start()
        {
            InstantiateBoids();

#if UNITY_EDITOR
            if (m_spawnDebugOnStart)
            {
                GameObject debugGO = new GameObject("DemoDebug");
                m_demoDebug = debugGO.AddComponent<DemoDebug>();
                m_demoDebug.SetWorldData(m_worldData.DefaultWorldRadius, m_spawnRadius);
            }
#endif
        }

        private void OnDestroy()
        {
            DisposeState();
        }

        public void ResetSimulation()
        {
            DestroyAllBoids();
            DisposeState();
            ResetVariables();
            InstantiateBoids();
        }
    
        public void SetCount(Slider slider)
        {
            m_count = (int)slider.value * m_worldData.DefaultMultiplier;
            m_spawnRadius = m_worldData.DefaultSpawnRadius * (slider.value * 0.1f);
        }

        private void ResetVariables()
        {
            m_cellSize = Mathf.Max(m_boidData.SeparationRadius, m_boidData.CohesionRadius, m_boidData.AlignmentRadius);
            
            m_rotations = new NativeArray<quaternion>(m_count, Allocator.Persistent);
            m_positions = new NativeArray<float3>(m_count, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_count, Allocator.Persistent);
            m_steerings = new NativeArray<float3>(m_count, Allocator.Persistent);
            m_probes = new NativeArray<float3>(m_count * DemoData.PROBES_PER_BOID, Allocator.Persistent);

            m_spatialGrid = new NativeParallelMultiHashMap<uint, int>(m_count, Allocator.Persistent);
        }

        private void DisposeState()
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
            m_boidParent = new GameObject("BoidParent");

            Transform[] transforms = new Transform[m_count];
            for (int index = 0; index < m_count; index++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * m_spawnRadius;
                Quaternion randomRotation = Random.rotationUniform;

                GameObject boid = Instantiate(m_boidData.Prefab, randomPosition, randomRotation, m_boidParent.transform);
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

        private void DestroyAllBoids()
        {
            Destroy(m_boidParent.gameObject);
        }

        private void Update()
        {
            m_boidsHandle.Complete();

#if UNITY_EDITOR
            m_demoDebug?.UpdateDebug(m_positions, m_velocities, m_steerings, m_probes);
#endif

            // Rebuilds the spatial hash grid with the boids position.
            m_spatialGrid.Clear();
            var spatialGridJob = new SpatialGridJob
            {
                Positions = m_positions,
                SpatialGrid = m_spatialGrid.AsParallelWriter(),
                CellSize = m_cellSize
            };

            // Updates position and rotation of probes based on boid position and velocity.
            var probesJob = new ProbesJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                ProbeLengthMultiplier = m_boidData.ProbeLengthMultiplier,
                ProbeAngle = m_boidData.ProbeAngle
            };

            // Overrides the steering vector to the calculate value (steering = value), because steering behaviours are
            // only accumulative for one frame.
            var flockSteeringJob = new FlockSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                SpatialGrid = m_spatialGrid,
                MaxPerceivedBoidsPerCell = m_perceivedBoidsPerCell, // We look for boids in a 3x3x3 Spatial Grid.
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
                SteeringAcceleration = m_boidData.FlockAcceleration
            };

            // Containment is calculated after flocking to ensure we know where the boid is headed. This increments
            // to the steering vector by the calculated value (steering += value).
            var containmentSteeringJob = new ContainmentSteeringJob
            {
                Positions = m_positions,
                Velocities = m_velocities,
                Probes = m_probes,
                Steerings = m_steerings,
                WorldRadius = m_worldData.DefaultWorldRadius,
                SteeringAcceleration = m_boidData.ContainmentAcceleration
            };

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

            JobHandle spatialGridHandle = spatialGridJob.Schedule(m_count, 64);
            JobHandle probeHandle = probesJob.Schedule(m_count, 64);
            JobHandle setupHandle = JobHandle.CombineDependencies(spatialGridHandle, probeHandle);
            JobHandle flockingSteeringHandle = flockSteeringJob.Schedule(m_count, 64, setupHandle);
            JobHandle containmentSteeringHandle =
            containmentSteeringJob.Schedule(m_count, 64, flockingSteeringHandle);
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

            [NativeDisableParallelForRestriction] public NativeArray<float3> Probes;

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
                Probes[index * DemoData.PROBES_PER_BOID] = originPosition + ray;

                // The remaining four are tilted upwards, downwards, leftwards and rightwards.
                quaternion upTilt = quaternion.AxisAngle(right, ProbeAngle);
                Probes[index * DemoData.PROBES_PER_BOID + 1] = originPosition + math.mul(upTilt, ray);

                quaternion downTilt = quaternion.AxisAngle(right, -ProbeAngle);
                Probes[index * DemoData.PROBES_PER_BOID + 2] = originPosition + math.mul(downTilt, ray);

                quaternion rightTilt = quaternion.AxisAngle(up, ProbeAngle);
                Probes[index * DemoData.PROBES_PER_BOID + 3] = originPosition + math.mul(rightTilt, ray);

                quaternion leftTilt = quaternion.AxisAngle(up, -ProbeAngle);
                Probes[index * DemoData.PROBES_PER_BOID + 4] = originPosition + math.mul(leftTilt, ray);
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
            public float SteeringAcceleration;
            
            public int MaxPerceivedBoidsPerCell;

            private struct Forces
            {
                public float3 SteeringVector;
                public float3 SeparationForce;
                public float3 CohesionForce;
                public float3 AlignmentForce;

                public int CohesionCount;
                public int AlignmentCount;
            }

            public void Execute(int index)
            {
                var forces = new Forces();
                var gridPosition = (int3)math.floor(Positions[index] / CellSize);
                var normalizedVelocity = math.normalizesafe(Velocities[index]);
                
                float maximumRadius = math.max(SeparationRadius, CohesionRadius);
                maximumRadius = math.max(AlignmentRadius, maximumRadius);

                SpatialPartitioning(index, gridPosition, normalizedVelocity, maximumRadius, ref forces);
                
                forces.SteeringVector += forces.SeparationForce * SeparationWeight;

                if (forces.CohesionCount > 0)
                {
                    forces.CohesionForce = forces.CohesionForce / forces.CohesionCount - Positions[index];
                    forces.SteeringVector += forces.CohesionForce * CohesionWeight;
                }

                if (forces.AlignmentCount > 0)
                {
                    forces.AlignmentForce = forces.AlignmentForce / forces.AlignmentCount;
                    forces.SteeringVector += forces.AlignmentForce * AlignmentWeight;
                }

                float steeringLengthSq = math.lengthsq(forces.SteeringVector);
                if (steeringLengthSq > SteeringAcceleration * SteeringAcceleration)
                {
                    forces.SteeringVector = math.normalizesafe(forces.SteeringVector) * SteeringAcceleration;
                }

                Steerings[index] = forces.SteeringVector;
            }

            private void SpatialPartitioning(int index, int3 gridPosition, float3 normalizedVelocity, float maximumRadius, ref Forces forces)
            {
                float3 position = Positions[index];

                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                for (int z = -1; z <= 1; z++)
                {
                    int3 otherGridPosition = gridPosition + new int3(x, y, z);
                    uint hash = GetHash(otherGridPosition);
                    int boidsPerCell = 0;

                    bool hasHashList = SpatialGrid.TryGetFirstValue(hash, out var other, out var iterator);
                    if (!hasHashList) continue;
                    
                    do
                    {
                        if (index == other) continue;
                        if (boidsPerCell > MaxPerceivedBoidsPerCell) break;
                        
                        float3 otherPosition = Positions[other];
                        
                        float3 distanceVector = otherPosition - position;
                        float distanceSq = math.lengthsq(distanceVector);

                        bool visible = distanceSq < maximumRadius * maximumRadius;
                        if (!visible) continue;
                        
                        float3 normalizeDistanceVector = math.normalizesafe(distanceVector);
                        float dot = math.dot(normalizedVelocity, normalizeDistanceVector);

                        if (distanceSq < SeparationRadius * SeparationRadius && dot > SeparationDot)
                        {
                            float distanceToNeighbour = math.sqrt(distanceSq);
                            forces.SeparationForce += -distanceVector / distanceToNeighbour;
                        }

                        if (distanceSq < CohesionRadius * CohesionRadius && dot > CohesionDot)
                        {
                            forces.CohesionForce += otherPosition;
                            forces.CohesionCount++;
                        }

                        if (distanceSq < AlignmentRadius * AlignmentRadius && dot > AlignmentDot)
                        {
                            forces.AlignmentForce += Velocities[other];
                            forces.AlignmentCount++;
                        }

                        boidsPerCell++;

                    } while (SpatialGrid.TryGetNextValue(out other, ref iterator));
                }
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
            public float SteeringAcceleration;

            public void Execute(int index)
            {
                float3 steering = new float3(0.0f);

                for (int i = 0; i < DemoData.PROBES_PER_BOID; i++)
                {
                    float3 probe = Probes[index * DemoData.PROBES_PER_BOID + i];
                    if (math.lengthsq(probe) < WorldRadius * WorldRadius) continue;

                    float probeLength = math.length(probe);
                    float ratioPenetration = (probeLength - WorldRadius) / probeLength;

                    float3 probeDirection = math.normalizesafe(probe - Positions[index]);
                    float3 collisionPoint = GetCollisionPoint(Positions[index], probeDirection, WorldRadius);

                    float3 collisionNormal = math.normalizesafe(-collisionPoint);
                    float3 velocityNormal = math.normalizesafe(-Velocities[index]);
                    float3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);

                    steering += perpendicular * ratioPenetration;
                }

                Steerings[index] += math.normalizesafe(steering) * SteeringAcceleration;
            }

            /// <summary>
            /// We simplified the Analytic Solution presented by Jean-Colas Prunier. This simplification outputs
            /// the correct position of a collision only when the ray origin is contained within the boundary radius.
            /// </summary>
            /// <a href="https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html">Ray-Sphere Intersection</a>
            private float3 GetCollisionPoint(float3 rayOrigin, float3 rayDirection, float colliderRadius)
            {
                if (math.lengthsq(rayOrigin) > WorldRadius * WorldRadius)
                {
                    rayOrigin = math.normalizesafe(rayOrigin) * WorldRadius;
                }

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

                float q = b > 0 ? -0.5f * (b + math.sqrt(discriminant)) : -0.5f * (b - math.sqrt(discriminant));

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
                return math.normalizesafe(collisionNormal - dotVector);
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
    }
}