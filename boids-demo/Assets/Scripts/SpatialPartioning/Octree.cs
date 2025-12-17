using System;
using System.Collections.Generic;
using UnityEngine;

namespace SpatialPartition
{
    public struct Unit : IComparable<Unit>
    {
        public uint Code;
        public int Index;

        public int CompareTo(Unit other)
        {
            return Code.CompareTo(other.Code);
        }
    }
    
    public class Octree
    {
        private readonly List<Unit> _octree;
        
        private readonly Rigidbody[] _bodies;
        private readonly VisionConfiguration _visionConfig;
        private readonly WorldConfiguration _worldConfig;

        public Octree(Rigidbody[] bodies, VisionConfiguration visionConfig, WorldConfiguration worldConfig)
        {
            _octree = new List<Unit>();

            _bodies = bodies;
            _visionConfig = visionConfig;
            _worldConfig = worldConfig;
        }

        public void Build()
        {
            _octree.Clear();
            PopulateTree();
            _octree.Sort();
        }

        private void PopulateTree()
        {
            for (int i = 0; i < _bodies.Length; i++)
            {
                _octree.Add(new Unit
                {
                    Code = GetMortonCode(_bodies[i].position),
                    Index = i
                });
            }
        }
        
        private uint GetMortonCode(Vector3 position)
        {
            uint z = (uint)(position.z / _visionConfig.VisionRadius + _worldConfig.CageRadius);
            uint y = (uint)(position.y / _visionConfig.VisionRadius + _worldConfig.CageRadius);
            uint x = (uint)(position.x / _visionConfig.VisionRadius + _worldConfig.CageRadius);
            
            uint code = (SpreadBits(z) << 2) | (SpreadBits(y) << 1) | SpreadBits(x); // interleave
            return code;
        }
        
        private uint SpreadBits(uint i)
        {
            i = (i | (i << 16)) & 0x030000ff;
            i = (i | (i << 8))  & 0x0300f00f;
            i = (i | (i << 4))  & 0x030c30c3;
            i = (i | (i << 2))  & 0x09249249;
            return i;
        }

        public void Query(List<Rigidbody> query, Vector3 minimum, Vector3 maximum)
        {
            query.Clear();
            uint begin = GetMortonCode(minimum);
            uint end = GetMortonCode(maximum);

            int index = BinarySearch(begin);
            while (index != -1 &&
                   index < _octree.Count &&
                   _octree[index].Code <= end)
            {
                var unit = _octree[index];
                var body = _bodies[unit.Index];
                if (IsInsideBoundary(body.position, minimum, maximum))
                {
                    query.Add(body);
                }
                index++;
            }
        }

        private bool IsInsideBoundary(Vector3 point, Vector3 minimum, Vector3 maximum)
        {
            return
                point.x < maximum.x && point.x > minimum.x &&
                point.y < maximum.y && point.y > minimum.y &&
                point.z < maximum.z && point.z > minimum.z;
        }

        private int BinarySearch(uint begin)
        {
            var low = 0;
            var high = _octree.Count - 1;
            var result = -1;
            
            while (low <= high)
            {
                var medium = (low + high) / 2;
                if (_octree[medium].Code >= begin)
                {
                    result = medium;
                    high = medium - 1;
                }
                else
                {
                    low = medium + 1;
                }
            }
            
            return result;
        }
    }
}