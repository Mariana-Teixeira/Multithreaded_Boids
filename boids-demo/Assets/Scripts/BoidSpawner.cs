using UnityEngine;
using Random = UnityEngine.Random;

public class BoidSpawner : MonoBehaviour
{
    [SerializeField] private GameObject _boidPrefab;
    [SerializeField] private Cage _cage;
    
    private readonly int _count = 50;
    private Rigidbody[] _boids;

    private void Awake()
    {
        _boids = new Rigidbody[_count];
    }

    private void Start()
    {
        for (int i = 0; i < _count; i++)
        {
            var randomPosition = Random.insideUnitSphere * Random.Range(0, _cage.CageRadius);
            var randomRotation = Random.rotation;
            var boidGO = Instantiate(_boidPrefab, randomPosition, randomRotation);
            _boids[i] = boidGO.GetComponent<Rigidbody>();
            boidGO.GetComponent<Boid>().AllBoids = _boids;
            boidGO.GetComponent<Boid>().Cage = _cage;
        }
    }
}
