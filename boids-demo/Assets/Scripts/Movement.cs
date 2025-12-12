using UnityEngine;

public class Movement
{
    private readonly Rigidbody _body;
    private readonly MovementConfiguration _movementConfig;

    public Movement(Rigidbody body, MovementConfiguration movementConfig)
    {
        _movementConfig = movementConfig;
        _body = body;
    }

    public float GetCurrentVelocity => _body.linearVelocity.magnitude;

    public float GetRandomSpeed() => Random.Range(_movementConfig.MinSpeed, _movementConfig.MaxSpeed);
    public void SetRandomVelocity(Vector3 forwardVector)
    {
        _body.linearVelocity = forwardVector * GetRandomSpeed();
    }
    
    public void Move(Vector3 steeringVector)
    {
        _body.linearVelocity += steeringVector * Time.deltaTime; 
        _body.linearVelocity = Vector3.ClampMagnitude(_body.linearVelocity, _movementConfig.MaxSpeed);
    }
    
    public void Rotate()
    {
        Quaternion targetRotation = Quaternion.LookRotation(_body.linearVelocity);
        _body.rotation = Quaternion.Slerp(_body.rotation, targetRotation, Time.fixedDeltaTime * _movementConfig.RotationSpeed);
    }
}