using Boids.Jobs;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

namespace Boids
{
    public class DemoManager : MonoBehaviour
    {
        [SerializeField] private Settings m_settings;
        
        private NativeParallelMultiHashMap<int, int> m_spatialMap;
        private TransformAccessArray m_transformArray;
        private NativeArray<float3> m_positions;
        private NativeArray<float3> m_velocities;
        private NativeArray<float3> m_steerings;

        private JobHandle m_jobHandle;

        private void Awake()
        {
            m_spatialMap = new NativeParallelMultiHashMap<int, int>(m_settings.BoidCount, Allocator.Persistent);
            m_positions = new NativeArray<float3>(m_settings.BoidCount, Allocator.Persistent);
            m_velocities = new NativeArray<float3>(m_settings.BoidCount, Allocator.Persistent);
            m_steerings = new NativeArray<float3>(m_settings.BoidCount, Allocator.Persistent);
            
            m_settings.VisionThreshold = math.cos(m_settings.VisionAngle * 0.5f * math.TORADIANS);
        }

        private void Start()
        {
            Transform[] transforms = new Transform[m_settings.BoidCount];
            for (int i = 0; i < m_settings.BoidCount; i++)
            {
                Transform boidTransform = Spawner.Spawn(m_settings.GridCenter, m_settings.GridRadius, m_settings.BoidPrefab);
                transforms[i] = boidTransform;
                m_positions[i] = boidTransform.position;
                m_velocities[i] = boidTransform.forward * m_settings.MinSpeed;
            }
            m_transformArray = new TransformAccessArray(transforms);
        }

        private void Update()
        {
            m_spatialMap.Clear();

            SpatialMapJob spatialMapJob = new SpatialMapJob
            {
                Positions = m_positions,
                CellSize = m_settings.CellRadius,
                SpatialMap = m_spatialMap.AsParallelWriter()
            };
            JobHandle spatialHandle = spatialMapJob.Schedule(m_settings.BoidCount, 64);
            
            BoidSteerJob steerJob = new BoidSteerJob
            {
                SpatialMap = m_spatialMap,
                GridCenter = m_settings.GridCenter,
                GridRadius = m_settings.GridRadius,
                CellSize = m_settings.CellRadius,
                VisionRadius = m_settings.VisionRadius,
                VisionThreshold = m_settings.VisionThreshold,
                SpringWeight = m_settings.SpringWeight,
                AlignmentWeight = m_settings.AlignmentWeight,
                SeparationWeight = m_settings.SeparationWeight,
                CohesionWeight = m_settings.CohesionWeight,
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings
            };
            JobHandle steerHandle = steerJob.Schedule(m_settings.BoidCount, 32, spatialHandle);
            
            BoidMoveJob moveJob = new BoidMoveJob
            {
                DeltaTime = Time.deltaTime,
                MinSpeed = m_settings.MinSpeed,
                MaxSpeed = m_settings.MaxSpeed,
                RotationSpeed = m_settings.RotationSpeed,
                Positions = m_positions,
                Velocities = m_velocities,
                Steerings = m_steerings
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
            m_positions.Dispose();
            m_velocities.Dispose();
            m_steerings.Dispose();
        }
    }
}