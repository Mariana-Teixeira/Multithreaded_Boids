using SpatialPartition;
using UnityEngine;
using Random = UnityEngine.Random;

public class BoidSpawner : MonoBehaviour
{
    [SerializeField] private BoidConfiguration _boidConfig;
    [SerializeField] private GameObject _boidPrefab;
    [SerializeField] private int _count = 50;
    
    private Rigidbody[] _bodies;
    private Octree _octree;

    private void Awake()
    {
        _bodies = new Rigidbody[_count];
        _octree = new Octree(_bodies, _boidConfig.Vision, _boidConfig.World);
    }

    private void Start()
    {
        SpawnBoids();
        _octree.Build();
    }

    private void SpawnBoids()
    {
        for (int i = 0; i < _count; i++)
        {
            var randomPosition = _boidConfig.World.CageCenter + Random.insideUnitSphere * Random.Range(0, _boidConfig.World.CageRadius);
            var randomRotation = Random.rotation;
            var boidGO = Instantiate(_boidPrefab, randomPosition, randomRotation);
            _bodies[i] = boidGO.GetComponent<Rigidbody>();
            boidGO.GetComponent<Boid>().Initialize(_bodies, _bodies[i], _octree, _boidConfig);
        }
    }

    private void OnDrawGizmos()
    {
        if (_boidConfig.World == null) return;
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(_boidConfig.World.CageCenter, _boidConfig.World.CageRadius);
    }
}
