using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Pool;
using Random = UnityEngine.Random;

public class Spawner : MonoBehaviour
{
    private IObjectPool<Rigidbody2D> _enemyPool;

    [SerializeField] private GameObject _enemyPrefab;
    [SerializeField] private int _defaultPoolSize = 50;
    [SerializeField] private int _maxPoolSize = 200;

    [SerializeField] private float _maxDistance;
    [SerializeField] private float _minDistance;

    [SerializeField] private float _spawnInterval = 2.0f;
    [SerializeField] private int _minGroupSize;
    [SerializeField] private int _maxGroupSize;

    private Coroutine _spawnRoutine;
    private WaitForSeconds _spawnWaitForSeconds;

    private void Awake()
    {
        _enemyPool = new ObjectPool<Rigidbody2D>(
            OnCreateEnemy,
            OnGetEnemy,
            OnReleaseEnemy,
            OnDestroyEnemy,
            defaultCapacity: _defaultPoolSize,
            maxSize: _maxPoolSize);
        
        _spawnWaitForSeconds = new WaitForSeconds(_spawnInterval);
    }

    private void Start()
    {
        _spawnRoutine = StartCoroutine(SpawnRoutine());
    }

    private void OnDestroy()
    {
        StopCoroutine(_spawnRoutine);
        _enemyPool.Clear();
    }

    private Rigidbody2D OnCreateEnemy()
    {
        GameObject enemyGO = Instantiate(_enemyPrefab);
        return enemyGO.GetComponent<Rigidbody2D>();
    }

    private void OnGetEnemy(Rigidbody2D enemy)
    {
        enemy.position = GetRandomPoint();
        enemy.gameObject.SetActive(true);
    }

    private void OnReleaseEnemy(Rigidbody2D enemy)
    {
        enemy.gameObject.SetActive(false);
    }

    private void OnDestroyEnemy(Rigidbody2D enemy)
    {
        Destroy(enemy.gameObject);
    }

    private Vector2 GetRandomPoint()
    {
        Vector2 randomDirection = Random.insideUnitCircle.normalized;
        float distance = Random.Range(_minDistance, _maxDistance);
        Vector2 randomPosition = PlayerTargetService.TargetPosition + randomDirection * distance;
        return randomPosition;
    }

    private IEnumerator SpawnRoutine()
    {
        while (true)
        {
            int enemyCount = Random.Range(_minGroupSize, _maxGroupSize);
            for (int i = 0; i < enemyCount; i++) _enemyPool.Get();
            yield return _spawnWaitForSeconds;
        }
    }
}
