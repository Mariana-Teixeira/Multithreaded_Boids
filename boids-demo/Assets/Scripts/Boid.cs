using System;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Boid : MonoBehaviour
{
    private Movement _movement;
    private Steering _steering;
    private Vision _vision;

    public void Initialize(Rigidbody[] allBodies, Rigidbody body, BoidConfiguration config)
    {
        _movement = new Movement(body, config.Movement);
        _steering = new Steering(config.Steering, config.World);
        _vision = new Vision(allBodies, config.Vision);

        OnInitialize();
    }

    private void OnInitialize()
    {
        _movement.SetRandomVelocity(transform.forward);
    }

    private void Update()
    {
        var visibleBodies = _vision.GetVisibleBodies(transform.position, transform.forward);
        var currentVelocity = _movement.GetCurrentVelocity;
        _steering.UpdateSteering(transform.position, transform.forward, currentVelocity, visibleBodies);
    }

    private void FixedUpdate()
    {
        _movement.Move(_steering.SteeringVector);
        _movement.Rotate();
    }
}