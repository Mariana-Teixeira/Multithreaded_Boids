using System;
using UnityEngine;
using Random = UnityEngine.Random;

public class BoidSpawner : MonoBehaviour
{
    [SerializeField] private BoidConfiguration _boidConfig;
    [SerializeField] private GameObject _boidPrefab;
    [SerializeField] private int _count = 50;
    
    private Rigidbody[] _bodies;
    private SpatialGrid _spatialGrid;

    private void Awake()
    {
        _bodies = new Rigidbody[_count];
        _spatialGrid = new SpatialGrid(_boidConfig.World);
    }

    private void Start()
    {
        SpawnBoids();
    }

    private void Update()
    {
        _spatialGrid.Refresh(_bodies);
    }

    private void SpawnBoids()
    {
        for (int i = 0; i < _count; i++)
        {
            var randomPosition = _boidConfig.World.GridCenter + Random.insideUnitSphere * Random.Range(0, _boidConfig.World.GridRadius);
            var randomRotation = Random.rotation;
            var boidGO = Instantiate(_boidPrefab, randomPosition, randomRotation);
            _bodies[i] = boidGO.GetComponent<Rigidbody>();
            boidGO.GetComponent<Boid>().Initialize(_spatialGrid, _bodies[i], _boidConfig);
        }
    }

    private void OnDrawGizmos()
    {
        if (_boidConfig.World == null) return;
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(_boidConfig.World.GridCenter, _boidConfig.World.GridRadius);
    }
}
