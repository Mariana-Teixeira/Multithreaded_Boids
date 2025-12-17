using System.Collections.Generic;
using SpatialPartition;
using UnityEngine;

public class Vision
{
    private readonly Octree _octree;
    private readonly VisionConfiguration _visionConfig;
    private readonly float _visionRadians;
    
    public Vision(Octree octree, VisionConfiguration visionConfig)
    {
        _octree = octree;
        _visionConfig = visionConfig;
        _visionRadians = Mathf.Cos(_visionConfig.VisionAngle * Mathf.Deg2Rad);
    }
    
    public List<Rigidbody> GetVisibleBodies(Vector3 currentPosition, Vector3 forwardVector)
    {
        Vector3 minBounds = new Vector3(
            currentPosition.x - _visionConfig.VisionRadius,
            currentPosition.y - _visionConfig.VisionRadius,
            currentPosition.z - _visionConfig.VisionRadius);
        Vector3 maxBounds = new Vector3(
            currentPosition.x + _visionConfig.VisionRadius,
            currentPosition.y + _visionConfig.VisionRadius,
            currentPosition.z + _visionConfig.VisionRadius);
        
        List<Rigidbody> queryBodies = new();
        List<Rigidbody> visibleBodies = new();
        _octree.Query(queryBodies, minBounds, maxBounds);
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