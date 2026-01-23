using Unity.Burst;
using Unity.Mathematics;

namespace Demo.Utilities
{
    /// <summary>
    /// Methods that prevent NaN errors from occuring during edge cases — such as when velocity magnitude is zero.
    /// Where the Mathematics Library would throw a DivideByZeroException, these return a fallback value.
    /// </summary>
    [BurstCompile]
    public static class CustomMath
    {
        /// <returns>Returns a zero when a NaN error would be thrown.</returns>
        public static float3 Normalize(float3 vector)
        {
            return math.lengthsq(vector) > math.EPSILON ? math.normalize(vector) : float3.zero;
        }

        /// <returns>Returns a zero when a NaN error would be thrown.</returns>
        public static float Length(float3 vector)
        {
            return math.lengthsq(vector) > math.EPSILON ? math.length(vector) : 0.0f;
        }

        /// <returns>Returns a zero when a NaN error would be thrown.</returns>
        public static float3 Divide(float3 numerator, float denominator)
        {
            return math.abs(denominator) < math.EPSILON ? float3.zero : numerator / denominator;
        }

        /// <returns>Returns a identity when a NaN error would be thrown.</returns>
        public static quaternion LookRotation(float3 forward, float3 up)
        {
            return math.lengthsq(forward) < math.EPSILON
                ? quaternion.identity
                : quaternion.LookRotation(forward, up);
        }
    }
}