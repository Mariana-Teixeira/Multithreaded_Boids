using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Animations;

public class Boid : MonoBehaviour
{
    private Rigidbody _body;
    private Vector3 _steeringForce;

    private readonly float _visionAngle = 60.0f; // Half the vision cone angle.
    private readonly float _visionRadius = 10.0f;
    private float VisionRadian => Mathf.Cos(_visionAngle * Mathf.Deg2Rad);

    public Rigidbody[] AllBoids { get; set; }
    private readonly List<Rigidbody> _visibleBoids = new();
    
    public Cage Cage { get; set; }

    private readonly float _maxSpeed = 6.0f;
    private readonly float _rotationSpeed = 6.0f;
    private readonly float _centeringForce = 8.0f;
    private readonly float _alignmentForce = 1.0f;
    private readonly float _separationForce = 2.0f;
    private readonly float _cohesionForce = 1.0f;
    
    private void Awake()
    {
        _body = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        ClearData();
        UpdateVision();
        UpdateCentering();
        UpdateAlignment();
        UpdateSeparation();
        UpdateCohesion();
        Move();
        LookAt();
    }

    private void ClearData()
    {
        _steeringForce = transform.forward;
        _visibleBoids.Clear();
    }

    private void UpdateVision()
    {
        foreach (var boid in AllBoids)
        {
            Vector3 vectorToBoid = boid.transform.position - transform.position;
            float dotProduct = Vector3.Dot(transform.forward, vectorToBoid.normalized);
            if (vectorToBoid.magnitude < _visionRadius && dotProduct > VisionRadian)
            {
                _visibleBoids.Add(boid);
            }
        }
    }
    
    private void UpdateCentering()
    {
        var bodyToCenter = Cage.transform.position - transform.position;
        var magnitudeToEdge = Cage.CageRadius - bodyToCenter.magnitude;
        var centeringRepulsion = 1 / magnitudeToEdge;
        var boidForce = bodyToCenter.normalized * centeringRepulsion;
        _steeringForce += boidForce * _centeringForce;

    }

    private void UpdateAlignment()
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in _visibleBoids)
        {
            boidForces += boid.linearVelocity;
        }
        _steeringForce += boidForces.normalized * _alignmentForce;
    }
    
    private void UpdateSeparation()
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in _visibleBoids)
        {
            var pushBoid = transform.position - boid.position;
            var pushRepulsion = 1 / pushBoid.magnitude;
            boidForces += pushBoid.normalized * pushRepulsion;
        }
        _steeringForce += boidForces.normalized * _separationForce;
    }

    private void UpdateCohesion()
    {
        Vector3 boidForces = Vector3.zero;
        foreach (var boid in _visibleBoids)
        {
            var pullBoid = boid.position - transform.position;
            boidForces += pullBoid.normalized;
        }
        _steeringForce += boidForces.normalized * _cohesionForce;
    }

    private void Move()
    {
        _body.linearVelocity += _steeringForce * Time.deltaTime; 
        _body.linearVelocity = Vector3.ClampMagnitude(_body.linearVelocity, _maxSpeed);
    }

    private void LookAt()
    {
        Quaternion targetRotation = Quaternion.LookRotation(_body.linearVelocity);
        _body.rotation = Quaternion.Slerp(_body.rotation, targetRotation, Time.fixedDeltaTime * _rotationSpeed);
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, _visionRadius);
        foreach (var boid in _visibleBoids)
        {
            Gizmos.DrawLine(transform.position, boid.transform.position);
        }
        
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + _steeringForce);
    }
}
