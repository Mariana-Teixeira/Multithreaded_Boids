using System.Collections.Generic;
using UnityEngine;

public class Steering
{
    private readonly SteeringConfiguration _steeringConfig;
    private readonly WorldConfiguration _worldConfig;
    
    public Vector3 SteeringVector { get; private set; }

    public Steering(SteeringConfiguration steeringConfiguration, WorldConfiguration worldConfig)
    {
        _steeringConfig = steeringConfiguration;
        _worldConfig = worldConfig;
    }
    
    public void UpdateSteering(
        Vector3 currentPosition, Vector3 forwardVector, float velocity,
        List<Rigidbody> visibleBodies)
    {
        SteeringVector = forwardVector;

        SteeringVector += GetSpringVector(velocity, currentPosition);
        SteeringVector += GetAlignmentVector(visibleBodies);
        SteeringVector += GetSeparationVector(currentPosition, visibleBodies);
        SteeringVector += GetCohesionVector(currentPosition, visibleBodies);
    }
    
    private Vector3 GetSpringVector(float velocity, Vector3 currentPosition) // F = -kx
    {
        var springConstant = velocity / _worldConfig.CageRadius;
        var bodyToCenter = _worldConfig.CageCenter - currentPosition;
        var boidForce = bodyToCenter.normalized * (springConstant * bodyToCenter.magnitude);
        return boidForce * _steeringConfig.SpringForce;
    }

    private Vector3 GetAlignmentVector(List<Rigidbody> visibleBodies)
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in visibleBodies)
        {
            boidForces += boid.linearVelocity;
        }
        return boidForces.normalized * _steeringConfig.AlignmentForce;
    }
    
    private Vector3 GetSeparationVector(Vector3 currentPosition, List<Rigidbody> visibleBodies)
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in visibleBodies)
        {
            var pushBoid = currentPosition - boid.position;
            var pushRepulsion = 1 / pushBoid.magnitude;
            boidForces += pushBoid.normalized * pushRepulsion;
        }
        return boidForces.normalized * _steeringConfig.SeparationForce;
    }

    private Vector3 GetCohesionVector(Vector3 currentPosition, List<Rigidbody> visibleBodies)
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in visibleBodies)
        {
            var pullBoid = boid.position - currentPosition;
            boidForces += pullBoid.normalized;
        }
        return boidForces.normalized * _steeringConfig.CohesionForce;
    }
}
