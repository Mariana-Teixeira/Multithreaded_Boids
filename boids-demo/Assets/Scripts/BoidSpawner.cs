using UnityEngine;
using Random = UnityEngine.Random;

public class BoidSpawner : MonoBehaviour
{
    [SerializeField] private BoidConfiguration _boidConfig;
    [SerializeField] private GameObject _boidPrefab;
    [SerializeField] private int _count = 50;
    
    private Rigidbody[] _boids;

    private void Awake()
    {
        _boids = new Rigidbody[_count];
    }

    private void Start()
    {
        for (int i = 0; i < _count; i++)
        {
            var randomPosition = _boidConfig.World.CageCenter + Random.insideUnitSphere * Random.Range(0, _boidConfig.World.CageRadius);
            var randomRotation = Random.rotation;
            var boidGO = Instantiate(_boidPrefab, randomPosition, randomRotation);
            _boids[i] = boidGO.GetComponent<Rigidbody>();
            boidGO.GetComponent<Boid>().Initialize(_boids, _boids[i], _boidConfig);
        }
    }

    private void OnDrawGizmos()
    {
        if (_boidConfig.World == null) return;
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(_boidConfig.World.CageCenter, _boidConfig.World.CageRadius);
    }
}
