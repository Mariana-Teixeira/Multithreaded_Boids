using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Boid : MonoBehaviour
{
    private Movement _movement;
    private Steering _steering;
    private Vision _vision;
    private bool _hasInitialized;

    public void Initialize(SpatialGrid spatialGrid, Rigidbody body, BoidConfiguration config)
    {
        _movement = new Movement(body, config.Movement);
        _steering = new Steering(config.Steering, config.World);
        _vision = new Vision(spatialGrid, config.Vision);

        OnInitialize();
    }

    private void OnInitialize()
    {
        _movement.SetRandomVelocity(transform.forward);
        _hasInitialized = true;
    }

    private void Update()
    {
        if (!_hasInitialized) return;
        
        var visibleBodies = _vision.GetVisibleBodies(transform.position, transform.forward);
        var currentVelocity = _movement.GetCurrentVelocity;
        _steering.UpdateSteering(transform.position, transform.forward, currentVelocity, visibleBodies);
    }

    private void FixedUpdate()
    {
        if (!_hasInitialized) return;
        
        _movement.Move(_steering.SteeringVector);
        _movement.Rotate();
    }
}