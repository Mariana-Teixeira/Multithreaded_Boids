using Boids.Configurations;
using Boids.Jobs;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

namespace Boids
{
    public class BoidManager : MonoBehaviour
    {
        [SerializeField] private SpawnerConfiguration m_spawnerConfig;
        [SerializeField] private WorldConfiguration m_worldConfig;
        [SerializeField] private VisionConfiguration m_visionConfig;
        [SerializeField] private MovementConfiguration m_movementConfig;
        [SerializeField] private SteeringConfiguration m_steeringConfig;

        private NativeParallelMultiHashMap<int, int> m_spatialMap;
        private TransformAccessArray m_transformArray;
        private NativeArray<float3> m_steeringForces;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_positions;

        private JobHandle m_jobHandle;

        private void Awake()
        {
            m_spatialMap = new NativeParallelMultiHashMap<int, int>(m_spawnerConfig.Count, Allocator.Persistent);
            
            m_steeringForces = new NativeArray<float3>(m_spawnerConfig.Count, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_spawnerConfig.Count, Allocator.Persistent);
            m_positions = new NativeArray<float3>(m_spawnerConfig.Count, Allocator.Persistent);
        }

        private void Start()
        {
            Transform[] transforms = new Transform[m_spawnerConfig.Count];
            for (int i = 0; i < m_spawnerConfig.Count; i++)
            {
                Transform transform = Spawner.Spawn(m_spawnerConfig, m_worldConfig);
                transforms[i] = transform;
                m_positions[i] = transform.position;
                m_velocities[i] = transform.forward * m_movementConfig.MinSpeed;
            }
            m_transformArray = new TransformAccessArray(transforms);
        }

        private void Update()
        {
            m_spatialMap.Clear();

            SpatialMapJob spatialMapJob = new SpatialMapJob
            {
                Positions = m_positions,
                CellSize = m_worldConfig.CellRadius,
                SpatialMap = m_spatialMap.AsParallelWriter()
            };
            JobHandle spatialHandle = spatialMapJob.Schedule(m_spawnerConfig.Count, 64);
            
            BoidSteerJob steerJob = new BoidSteerJob
            {
                Velocities = m_velocities,
                Positions = m_positions,
                SpatialMap = m_spatialMap,
                GridCenter = m_worldConfig.GridCenter,
                GridRadius = m_worldConfig.GridRadius,
                CellSize = m_worldConfig.CellRadius,
                VisionRadius = m_visionConfig.VisionAngle,
                SpringWeight = m_steeringConfig.SpringWeight,
                AlignmentWeight = m_steeringConfig.AlignmentWeight,
                SeparationWeight = m_steeringConfig.SeparationWeight,
                CohesionWeight = m_steeringConfig.CohesionWeight,
                SteeringForces = m_steeringForces
            };
            JobHandle steerHandle = steerJob.Schedule(m_spawnerConfig.Count, 32, spatialHandle);
            
            BoidMoveJob moveJob = new BoidMoveJob
            {
                DeltaTime = Time.deltaTime,
                MinSpeed = m_movementConfig.MinSpeed,
                MaxSpeed = m_movementConfig.MaxSpeed,
                RotationSpeed = m_movementConfig.RotationSpeed,
                SteeringForces = m_steeringForces,
                Velocities = m_velocities,
                Positions = m_positions
            };
            m_jobHandle = moveJob.Schedule(m_transformArray, steerHandle);
        }

        private void LateUpdate()
        {
            m_jobHandle.Complete();
        }

        private void OnDestroy()
        {
            m_spatialMap.Dispose();
            m_transformArray.Dispose();
            m_steeringForces.Dispose();
            m_velocities.Dispose();
            m_positions.Dispose();
        }
    }
}