using System.Collections.Generic;
using UnityEngine;

public class SpatialGrid
{
    private readonly WorldConfiguration _worldConfig;
    private readonly float _cellSize;
    private readonly int _gridSize;

    private int _gridArea => _gridSize * _gridSize * _gridSize;
    private readonly List<Rigidbody>[] _buckets;

    public SpatialGrid(WorldConfiguration worldConfig)
    {
        _worldConfig = worldConfig;
        _cellSize = worldConfig.CellRadius;
        _gridSize = (int)(worldConfig.GridDiameter / worldConfig.CellRadius);

        _buckets = new List<Rigidbody>[_gridArea];
        for (int i = 0; i < _buckets.Length; i++)
            _buckets[i] = new List<Rigidbody>();
    }

    private void AddBody(Rigidbody body)
    {
        Vector3Int gridPosition = GetGridPosition(body.position);
        bool valid = GetIndex(gridPosition, out int index);
        if (!valid) return;
        _buckets[index].Add(body);
    }

    private Vector3Int GetGridPosition(Vector3 worldPosition)
    {
        var offset = _worldConfig.GridCenter - Vector3.one * _worldConfig.GridRadius;
        Vector3 localPosition = worldPosition - offset;
        
        return new Vector3Int(
            (int)(localPosition.x / _cellSize),
            (int)(localPosition.y / _cellSize),
            (int)(localPosition.z / _cellSize));
    }

    private bool GetIndex(Vector3Int gridPosition, out int index)
    {
        index = gridPosition.x + gridPosition.y * _gridSize + gridPosition.z * _gridSize * _gridSize;
        return index >= 0 && index < _gridArea;
    }

    public void Refresh(Rigidbody[] bodies)
    {
        foreach (var list in _buckets)
            list.Clear();

        foreach (var body in bodies)
            AddBody(body);
    }

    public List<Rigidbody> FindNearby(Vector3 position)
    {
        List<Rigidbody> nearby = new List<Rigidbody>();
        Vector3Int gridPosition = GetGridPosition(position);
        for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        for (int z = -1; z <= 1; z++)
        {
            Vector3Int nearbyGridPosition = gridPosition - new Vector3Int(x, y, z);
            bool valid = GetIndex(nearbyGridPosition, out int index);
            if (!valid) continue;
            nearby.AddRange(_buckets[index]);
        }
        return nearby;
    }
}