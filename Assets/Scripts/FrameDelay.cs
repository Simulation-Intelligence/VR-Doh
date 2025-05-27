using System.Collections.Generic;
using UnityEngine;

public class LongTermFrameDelay : MonoBehaviour
{
    private Queue<float> frameTimes = new Queue<float>(); // 存储每帧时间的队列
    private float totalFrameTime = 0f; // 累积帧时间
    private const float timeWindow = 5f; // 时间窗口，过去5秒
    private float timeSinceLastOutput = 0f; // 用于每秒输出一次

    void Update()
    {
        // 每帧的时间间隔
        float currentDeltaTime = Time.deltaTime;
        frameTimes.Enqueue(currentDeltaTime);
        totalFrameTime += currentDeltaTime;

        // 移除超出5秒范围的帧时间
        while (totalFrameTime > timeWindow && frameTimes.Count > 0)
        {
            totalFrameTime -= frameTimes.Dequeue();
        }

        // 计算时间间隔，用于控制每秒输出一次
        timeSinceLastOutput += currentDeltaTime;
        if (timeSinceLastOutput >= 1f)
        {
            timeSinceLastOutput = 0f; // 重置计时

            // 计算过去5秒的平均帧延迟（秒）
            float averageFrameTime = totalFrameTime / frameTimes.Count;

            // 转换为毫秒显示
            float averageFrameTimeMs = averageFrameTime * 1000.0f;

            // 计算FPS
            float averageFPS = 1.0f / averageFrameTime;

            // 输出过去5秒的平均帧延迟和FPS
            Debug.Log($"过去5秒平均帧延迟: {averageFrameTimeMs} 毫秒");
            Debug.Log($"过去5秒平均FPS: {averageFPS}");
        }
    }
}
