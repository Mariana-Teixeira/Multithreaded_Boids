using System.Collections.Generic;
using UnityEngine;

public class Vision
{
    private readonly SpatialGrid _spatialGrid;
    private readonly VisionConfiguration _visionConfig;
    private readonly float _visionRadians;

    private List<Rigidbody> _queryBodies;
    private List<Rigidbody> _visibleBodies;
    
    public Vision(SpatialGrid spatialGrid, VisionConfiguration visionConfig)
    {
        _spatialGrid = spatialGrid;
        _visionConfig = visionConfig;
        _visionRadians = Mathf.Cos(_visionConfig.VisionAngle * Mathf.Deg2Rad);

        _queryBodies = new List<Rigidbody>();
        _visibleBodies = new List<Rigidbody>();
    }
    
    public List<Rigidbody> GetVisibleBodies(Vector3 currentPosition, Vector3 forwardVector)
    {
        _visibleBodies.Clear();
        _queryBodies = _spatialGrid.FindNearby(currentPosition);
        foreach (var body in _queryBodies)
        {
            Vector3 vectorToBody = body.transform.position - currentPosition;
            float dotProduct = Vector3.Dot(forwardVector, vectorToBody.normalized);
            if (dotProduct < _visionRadians)
            {
                _visibleBodies.Add(body);
            }
        }
        return _visibleBodies;
    }
}