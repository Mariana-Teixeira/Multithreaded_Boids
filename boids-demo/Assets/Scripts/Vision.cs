using System.Collections.Generic;
using UnityEngine;

public class Vision
{
    private readonly SpatialGrid _spatialGrid;
    private readonly VisionConfiguration _visionConfig;
    private readonly float _visionRadians;
    
    public Vision(SpatialGrid spatialGrid, VisionConfiguration visionConfig)
    {
        _spatialGrid = spatialGrid;
        _visionConfig = visionConfig;
        _visionRadians = Mathf.Cos(_visionConfig.VisionAngle * Mathf.Deg2Rad);
    }
    
    public List<Rigidbody> GetVisibleBodies(Vector3 currentPosition, Vector3 forwardVector)
    {
        List<Rigidbody> visibleBodies = new();
        List<Rigidbody> queryBodies = _spatialGrid.FindNearby(currentPosition);
        foreach (var body in queryBodies)
        {
            Vector3 vectorToBody = body.transform.position - currentPosition;
            float dotProduct = Vector3.Dot(forwardVector, vectorToBody.normalized);
            if (dotProduct < _visionRadians)
            {
                visibleBodies.Add(body);
            }
        }
        return visibleBodies;
    }
}