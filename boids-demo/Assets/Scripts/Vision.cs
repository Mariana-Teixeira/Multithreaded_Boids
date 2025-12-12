using System.Collections.Generic;
using UnityEngine;

public class Vision
{
    private readonly Rigidbody[] _allBodies;
    private readonly VisionConfiguration _visionConfig;
    private readonly float _visionRadians;
    
    public Vision(Rigidbody[] allBodies, VisionConfiguration visionConfig)
    {
        _allBodies = allBodies;
        _visionConfig = visionConfig;
        _visionRadians = Mathf.Cos(_visionConfig.VisionAngle * Mathf.Deg2Rad);
    }
    
    public List<Rigidbody> GetVisibleBodies(Vector3 currentPosition, Vector3 forwardVector)
    {
        List<Rigidbody> visibleBodies = new();
        foreach (var body in _allBodies)
        {
            Vector3 vectorToBody = body.transform.position - currentPosition;
            float dotProduct = Vector3.Dot(forwardVector, vectorToBody.normalized);
            if (vectorToBody.magnitude < _visionConfig.VisionRadius && dotProduct > _visionRadians)
            {
                visibleBodies.Add(body);
            }
        }
        return visibleBodies;
    }
}